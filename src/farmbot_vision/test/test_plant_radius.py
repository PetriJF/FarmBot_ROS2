"""Unit tests for the plant radius measurement function, run without rclpy."""
import asyncio
from types import SimpleNamespace

import cv2

from farmbot_vision.plant_radius import green_mask, measure_plant_radius_px, PlantRadiusNode

import numpy as np

HSV_MIN = [40, 50, 50]
HSV_MAX = [90, 255, 255]
GREEN_BGR = (0, 200, 0)


def _mask_for(image: np.ndarray) -> np.ndarray:
    """Build the green mask for a synthetic test image using the module's default bounds."""
    return green_mask(image, HSV_MIN, HSV_MAX)


def test_measure_plant_radius_of_single_centred_blob():
    """A single green blob centred in frame should measure close to its drawn pixel radius."""
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (320, 240), 40, GREEN_BGR, -1)

    radius_px = measure_plant_radius_px(_mask_for(image), min_area_px=100)
    assert abs(radius_px - 40) <= 2


def test_measure_plant_radius_merges_all_blobs():
    """Multiple green blobs (a fragmented plant) are combined into one measurement."""
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (200, 240), 15, GREEN_BGR, -1)
    cv2.circle(image, (450, 240), 15, GREEN_BGR, -1)

    radius_px = measure_plant_radius_px(_mask_for(image), min_area_px=100)
    # The far blob's edge sits ~145 px from the frame centre (320, 240) - far beyond either
    # blob's own radius (15 px), confirming both were combined into one measurement.
    assert radius_px > 100


def test_measure_plant_radius_no_green_returns_none():
    """Bare soil with no green pixels should yield no measurement."""
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    assert measure_plant_radius_px(_mask_for(image), min_area_px=100) is None


def test_measure_callback_reports_map_update_failure():
    """A measured radius is not a successful pipeline result unless it is saved."""
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.circle(image, (50, 50), 20, GREEN_BGR, -1)

    class CaptureClient:
        """Return a successful synthetic camera capture."""

        @staticmethod
        def service_is_ready():
            """Report that the synthetic service is available."""
            return True

        @staticmethod
        async def call_async(_request):
            """Return a tightly packed bgr8 image like standard_camera does."""
            image_msg = SimpleNamespace(data=image.tobytes(), height=100, width=100)
            return SimpleNamespace(success=True, message='', image=image_msg)

    async def failed_map_update(_index, _radius):
        """Simulate a rejected map update."""
        return False

    node = SimpleNamespace(
        capture_client=CaptureClient(),
        hsv_min=HSV_MIN,
        hsv_max=HSV_MAX,
        min_contour_area_px=100,
        mm_per_pixel=0.5,
        plant_radius_padding_mm=20.0,
        update_map=failed_map_update,
    )
    request = SimpleNamespace(index=99)
    response = SimpleNamespace(success=False, message='', plant_radius=0.0)

    result = asyncio.run(PlantRadiusNode.measure_callback(node, request, response))

    assert result.success is False
    assert result.message == 'Plant radius measured but map update failed'
    assert result.plant_radius > 20.0
