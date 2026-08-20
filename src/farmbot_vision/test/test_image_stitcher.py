"""Unit tests for the image stitching placement and blending, run without rclpy."""
import asyncio
from types import SimpleNamespace

from farmbot_vision.image_stitcher import (canvas_size, feather_alpha, ImageStitcherNode,
                                           map_affine, stitch_frame)

import numpy as np

import pytest

BED_MM = (1000.0, 1000.0)
MAP_MM_PER_PX = 1.0
FRAME_SIZE = (100, 100)
FRAME_GREY = 200


def blank_canvas() -> np.ndarray:
    """Start an empty BGRA bed map, as the node does on its first stitch."""
    width, height = canvas_size(BED_MM, MAP_MM_PER_PX)
    return np.zeros((height, width, 4), dtype=np.uint8)


def grey_frame() -> np.ndarray:
    """Return a uniformly lit frame, so any variation on the map comes from blending alone."""
    return np.full((*FRAME_SIZE, 3), FRAME_GREY, dtype=np.uint8)


def aligned_mm_to_px(mm_per_px=1.0) -> np.ndarray:
    """Return a transform square with the bed, with the y axis flipped as a real camera's is."""
    return np.diag([1.0 / mm_per_px, -1.0 / mm_per_px])


def rotated_mm_to_px(rotation_deg, mm_per_px=1.0) -> np.ndarray:
    """Return the same, but with the camera mounted at an angle to the gantry."""
    angle = np.radians(rotation_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    return (rotation @ np.diag([1.0, -1.0])) / mm_per_px


def place(canvas, frame, camera_xy, mm_to_px=None, crop_fraction=1.0, feather_px=0):
    """Stitch one frame onto the canvas from the given camera position."""
    affine = map_affine(camera_xy, aligned_mm_to_px() if mm_to_px is None else mm_to_px,
                        frame.shape, MAP_MM_PER_PX, canvas.shape[0])
    stitch_frame(canvas, frame, affine, crop_fraction, feather_px)


def coverage_box(canvas) -> tuple:
    """Return the (left, top, right, bottom) bounds of everything with any coverage."""
    rows, columns = np.nonzero(canvas[..., 3])
    return columns.min(), rows.min(), columns.max(), rows.max()


def test_canvas_size_covers_the_whole_bed():
    """The canvas is sized from the bed dimensions, rounding up so no bed is left off."""
    assert canvas_size((2400.0, 1200.0), 0.5) == (4800, 2400)
    assert canvas_size((999.0, 999.0), 2.0) == (500, 500)


def test_frame_lands_at_the_commanded_bed_position():
    """A frame captured at a bed position is centred on that position in the map."""
    canvas = blank_canvas()

    place(canvas, grey_frame(), (500.0, 500.0))

    left, top, right, bottom = coverage_box(canvas)
    assert (left + right) / 2 == pytest.approx(500.0, abs=1.5)
    assert (top + bottom) / 2 == pytest.approx(500.0, abs=1.5)
    # 1mm/px camera onto a 1mm/px map, so the 100px frame stays 100 map px across.
    assert right - left == pytest.approx(98, abs=2)


def test_bed_y_axis_is_flipped_on_the_map():
    """Bed y grows upwards while image rows grow downwards, so low y sits low on the map."""
    canvas = blank_canvas()

    place(canvas, grey_frame(), (500.0, 200.0))

    _, top, _, bottom = coverage_box(canvas)
    assert (top + bottom) / 2 == pytest.approx(800.0, abs=1.5)


def test_rotated_camera_still_lands_on_the_commanded_position():
    """A camera mounted at an angle rotates the frame but does not move where it belongs."""
    canvas = blank_canvas()

    place(canvas, grey_frame(), (500.0, 500.0), mm_to_px=rotated_mm_to_px(30.0))

    alpha = canvas[..., 3].astype(float)
    rows, columns = np.nonzero(alpha)
    weights = alpha[rows, columns]
    assert np.average(columns, weights=weights) == pytest.approx(500.0, abs=1.5)
    assert np.average(rows, weights=weights) == pytest.approx(500.0, abs=1.5)


def test_frame_straddling_the_bed_edge_is_clipped():
    """Half a frame hanging off two bed edges is trimmed, not an error and not wrapped."""
    canvas = blank_canvas()

    place(canvas, grey_frame(), (20.0, 20.0))

    left, top, right, bottom = coverage_box(canvas)
    assert left == 0
    assert bottom == canvas.shape[0] - 1
    assert right < canvas.shape[1] - 1
    assert top > 0


def test_frame_entirely_off_the_bed_leaves_the_map_untouched():
    """A position well outside the bed writes nothing rather than clamping to an edge."""
    canvas = blank_canvas()

    place(canvas, grey_frame(), (-500.0, -500.0))

    assert not canvas[..., 3].any()


def test_overlapping_frames_blend_without_a_dark_seam():
    """Two feathered frames over the same uniform scene must stay that uniform brightness."""
    canvas = blank_canvas()

    place(canvas, grey_frame(), (480.0, 500.0), crop_fraction=0.9, feather_px=15)
    place(canvas, grey_frame(), (540.0, 500.0), crop_fraction=0.9, feather_px=15)

    covered = canvas[..., 3] > 25
    assert covered.any()
    # A naive blend would drag the second frame's feathered edge towards the blank canvas,
    # leaving a dark band down the overlap; normalised alpha-over holds the scene's own value.
    assert np.allclose(canvas[..., :3][covered], FRAME_GREY, atol=2)


def test_feather_alpha_fades_from_a_solid_centre_to_an_empty_border():
    """The blend weight is solid in the middle, zero outside the crop, and ramps between."""
    alpha = feather_alpha((100, 100, 3), crop_fraction=0.8, feather_px=5)

    assert alpha[50, 50] == pytest.approx(1.0)
    assert alpha[0, 0] == 0.0
    assert alpha[5, 50] == 0.0
    assert 0.0 < alpha[12, 50] < 1.0


def test_feather_alpha_without_feathering_is_a_hard_crop():
    """Turning the feather off leaves a plain cropped rectangle, for tuning comparisons."""
    alpha = feather_alpha((100, 100, 3), crop_fraction=0.8, feather_px=0)

    assert set(np.unique(alpha)) == {0.0, 1.0}
    assert alpha[50, 50] == 1.0
    assert alpha[5, 50] == 0.0


def stitcher_double(canvas, map_path, load_calibration=None):
    """Stand in for the node, with the camera and the map handler replaced by fixed answers."""
    frame = grey_frame()

    class CaptureClient:
        """Return a successful synthetic camera capture."""

        @staticmethod
        def service_is_ready():
            """Report that the synthetic service is available."""
            return True

        @staticmethod
        async def call_async(_request):
            """Return a tightly packed bgr8 image like standard_camera does."""
            image = SimpleNamespace(data=frame.tobytes(),
                                    height=FRAME_SIZE[0], width=FRAME_SIZE[1])
            return SimpleNamespace(success=True, message='', image=image)

    async def bed_size():
        """Report the bed dimensions the map handler would return."""
        return BED_MM

    return SimpleNamespace(
        capture_client=CaptureClient(),
        load_calibration=load_calibration or aligned_mm_to_px,
        bed_size_mm=bed_size,
        load_canvas=lambda _bed_size: canvas,
        map_path=map_path,
        map_mm_per_px=MAP_MM_PER_PX,
        feather_px=0,
        frame_crop_fraction=1.0,
        camera_offset_x=30.0,
        camera_offset_y=-40.0,
    )


def test_stitch_callback_places_the_frame_at_the_lens_not_the_gantry(tmp_path):
    """The fixed camera-to-gantry offset has to shift where the frame lands on the map."""
    canvas = blank_canvas()
    node = stitcher_double(canvas, map_path=str(tmp_path / 'bed_map.png'))
    request = SimpleNamespace(x=400.0, y=600.0)
    response = SimpleNamespace(success=False, message='')

    result = asyncio.run(ImageStitcherNode.stitch_callback(node, request, response))

    assert result.success is True
    left, top, right, bottom = coverage_box(canvas)
    # The lens sits at (400 + 30, 600 - 40) = (430, 560), and bed y flips onto the map.
    assert (left + right) / 2 == pytest.approx(430.0, abs=1.5)
    assert (top + bottom) / 2 == pytest.approx(BED_MM[1] - 560.0, abs=1.5)


def test_stitch_callback_reports_a_failed_write(tmp_path):
    """A failed cv2.imwrite (e.g. bad path, disk full) is reported, not silently ignored."""
    canvas = blank_canvas()
    unwritable_path = str(tmp_path / 'no_such_directory' / 'bed_map.png')
    node = stitcher_double(canvas, map_path=unwritable_path)
    request = SimpleNamespace(x=400.0, y=600.0)
    response = SimpleNamespace(success=False, message='')

    result = asyncio.run(ImageStitcherNode.stitch_callback(node, request, response))

    assert result.success is False
    assert unwritable_path in result.message


def test_stitch_callback_reports_a_missing_calibration(tmp_path):
    """Without a calibration the map is left untouched and the caller is told why."""
    def uncalibrated():
        """Fail the way the node does when no calibration file has been written yet."""
        raise ValueError('No calibration at /nowhere; run camera/calibrate first')

    canvas = blank_canvas()
    node = stitcher_double(canvas, map_path=str(tmp_path / 'bed_map.png'),
                           load_calibration=uncalibrated)
    request = SimpleNamespace(x=400.0, y=600.0)
    response = SimpleNamespace(success=False, message='')

    result = asyncio.run(ImageStitcherNode.stitch_callback(node, request, response))

    assert result.success is False
    assert 'camera/calibrate' in result.message
    assert not canvas[..., 3].any()
