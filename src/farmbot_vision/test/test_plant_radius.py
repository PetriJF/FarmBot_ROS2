"""Unit tests for the plant radius measurement function, run without rclpy."""
import cv2

from farmbot_vision.plant_radius import green_mask, measure_plant_radius_px

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
