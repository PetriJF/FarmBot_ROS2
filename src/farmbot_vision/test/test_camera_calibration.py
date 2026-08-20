"""Unit tests for the camera calibration maths and card detection, run without rclpy."""
import cv2

from farmbot_vision.camera_calibration import (CALIBRATION_OFFSETS_MM, find_circle_grid,
                                               PATTERN_SIZE, solve_calibration)

import numpy as np

import pytest

MM_PER_PX = 0.4
ROTATION_DEG = 12.0
FRAME_SIZE = (480, 640)
FRAME_CENTRE = (FRAME_SIZE[1] / 2.0, FRAME_SIZE[0] / 2.0)


def card_points_mm() -> np.ndarray:
    """Return the calibration card's circle centres in mm, in the order OpenCV reports them."""
    points = []
    for row in range(PATTERN_SIZE[1]):
        for col in range(PATTERN_SIZE[0]):
            points.append(((2 * col + row % 2) * 15.0, row * 15.0))
    return np.array(points)


def centred_card() -> np.ndarray:
    """Place the card so it stays in frame across all three calibration positions."""
    card = card_points_mm()
    return card + (np.mean(CALIBRATION_OFFSETS_MM, axis=0) - card.mean(axis=0))


def mm_to_px_matrix(rotation_deg=ROTATION_DEG, mm_per_px=MM_PER_PX) -> np.ndarray:
    """Build a mm-to-px transform with a rotation, a scale, and the camera's flipped y axis."""
    angle = np.radians(rotation_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    return (rotation @ np.diag([1.0, -1.0])) / mm_per_px


def project(points_mm, mm_to_px, camera_xy) -> np.ndarray:
    """Image the given bed points as a camera sitting at camera_xy would see them."""
    return (points_mm - np.array(camera_xy)) @ mm_to_px.T + np.array(FRAME_CENTRE)


def render_card(points_mm, mm_to_px, camera_xy, dark_dots=True) -> np.ndarray:
    """Draw the calibration card as the camera at camera_xy would photograph it."""
    background, dot = (255, (0, 0, 0)) if dark_dots else (0, (255, 255, 255))
    image = np.full((*FRAME_SIZE, 3), background, dtype=np.uint8)
    radius = int(round(5.0 / MM_PER_PX))
    for x, y in project(points_mm, mm_to_px, camera_xy):
        cv2.circle(image, (int(round(x)), int(round(y))), radius, dot, -1, cv2.LINE_AA)
    return image


def captures_of(points_mm, mm_to_px) -> list:
    """Project the card from each of the three calibration positions."""
    return [project(points_mm, mm_to_px, np.array(offset)) for offset in CALIBRATION_OFFSETS_MM]


def test_solve_recovers_a_known_transform():
    """Three captures at known gantry offsets pin down the mm-to-px transform exactly."""
    expected = mm_to_px_matrix()

    calibration = solve_calibration(captures_of(card_points_mm(), expected))

    assert np.allclose(calibration['mm_to_px'], expected)
    assert calibration['mm_per_px'] == pytest.approx(MM_PER_PX)
    assert calibration['rotation_deg'] == pytest.approx(ROTATION_DEG)


def test_solve_recovers_a_mirrored_camera_mounting():
    """A camera mounted so its x axis opposes the bed's is just a negative determinant."""
    expected = mm_to_px_matrix() @ np.diag([-1.0, 1.0])

    calibration = solve_calibration(captures_of(card_points_mm(), expected))

    assert np.allclose(calibration['mm_to_px'], expected)
    assert calibration['mm_per_px'] == pytest.approx(MM_PER_PX)


def test_card_geometry_agrees_with_the_solved_scale():
    """The card's own row length gives an independent scale that backs up the gantry solve."""
    calibration = solve_calibration(captures_of(card_points_mm(), mm_to_px_matrix()))

    assert calibration['mm_per_px_card'] == pytest.approx(calibration['mm_per_px'])


def test_solve_rejects_captures_that_never_moved():
    """A gantry that did not move leaves a singular transform, which must not be saved."""
    still = project(card_points_mm(), mm_to_px_matrix(), np.array([0.0, 0.0]))

    with pytest.raises(ValueError):
        solve_calibration([still, still, still])


def test_solve_rejects_the_wrong_number_of_captures():
    """Solving needs all three captures; two is a caller error, not a calibration."""
    frame = project(card_points_mm(), mm_to_px_matrix(), np.array([0.0, 0.0]))

    with pytest.raises(ValueError):
        solve_calibration([frame, frame])


def test_detects_the_card_at_either_polarity():
    """Both printed polarities are handled, since findCirclesGrid only ever sees dark blobs."""
    card = centred_card()

    for dark_dots in (True, False):
        image = render_card(card, mm_to_px_matrix(), np.array([0.0, 0.0]), dark_dots=dark_dots)
        assert find_circle_grid(image) is not None, f'dark_dots={dark_dots} not detected'


def test_detects_and_solves_from_rendered_frames():
    """The whole path - detect the grid in three rendered frames, then solve - round trips."""
    expected = mm_to_px_matrix()
    card = centred_card()

    captures = []
    for offset in CALIBRATION_OFFSETS_MM:
        centres = find_circle_grid(render_card(card, expected, np.array(offset)))
        assert centres is not None, f'card not detected at offset {offset}'
        captures.append(centres)

    calibration = solve_calibration(captures)

    assert np.allclose(calibration['mm_to_px'], expected, atol=0.05)
    assert calibration['mm_per_px'] == pytest.approx(MM_PER_PX, abs=0.005)
    assert calibration['mm_per_px_card'] == pytest.approx(MM_PER_PX, abs=0.005)


def test_no_card_in_frame_returns_none():
    """A frame of bare soil yields no detection rather than a bogus grid."""
    assert find_circle_grid(np.zeros((*FRAME_SIZE, 3), dtype=np.uint8)) is None
