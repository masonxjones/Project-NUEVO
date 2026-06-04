"""
ros2_ws/src/vision/vision/test_and_tune.py
───────────────────────────────────────────────────
Visual debugger for burger piece colour detection.

Run from the Project-NUEVO root:
    python3 ros2_ws/src/vision/vision/test_and_tune.py

Click on the image window to print HSV values at that pixel —
use these to tune the HSV ranges in detection_burger_pieces.py.
"""

import cv2
import numpy as np

from detection_burger_pieces import (
    classify_burger_pieces,
    BUN_YELLOW_LOW,
    BUN_YELLOW_HIGH,
    PATTY_RED_LOW_1,
    PATTY_RED_HIGH_1,
    PATTY_RED_LOW_2,
    PATTY_RED_HIGH_2,
)

# ── Absolute path to test image ───────────────────────────────────────────────
IMAGE_PATH = "/ros2_ws/src/vision/vision/IMG_6955.jpg"


def mouse_callback(event, x, y, flags, param):
    """Print HSV values and suggested tuning range when you click the image."""
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_img = param['hsv']
        h, s, v = hsv_img[y, x]
        print(f"Clicked (x={x}, y={y})  →  HSV: ({h}, {s}, {v})")
        print(f"  Suggested range: "
              f"Low({max(0,h-15)}, {max(0,s-40)}, {max(0,v-40)})  "
              f"High({min(180,h+15)}, 255, 255)\n")


def main():
    # 1. Load image
    crop = cv2.imread(IMAGE_PATH)
    if crop is None:
        print(f"Error: could not load image from '{IMAGE_PATH}'")
        print("Check that the path is correct and the file exists.")
        return

    print(f"Loaded image: {crop.shape[1]}x{crop.shape[0]} px")

    # 2. Classify
    result = classify_burger_pieces(crop)
    print("=" * 40)
    print("  CLASSIFICATION RESULTS")
    print("=" * 40)
    for k, v in result.items():
        print(f"  {k:14}: {v}")
    print("=" * 40)
    print("Click on 'Original Crop' window to print HSV values at that pixel.\n")

    # 3. Build HSV image for the mouse callback
    blurred = cv2.GaussianBlur(crop, (5, 5), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 4. Build all masks for visual debugging
    yellow_mask = cv2.inRange(hsv, BUN_YELLOW_LOW, BUN_YELLOW_HIGH)

    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, PATTY_RED_LOW_1, PATTY_RED_HIGH_1),
        cv2.inRange(hsv, PATTY_RED_LOW_2, PATTY_RED_HIGH_2),
    )

    # 5. Show windows
    cv2.namedWindow("Original Crop")
    cv2.setMouseCallback("Original Crop", mouse_callback, param={'hsv': hsv})

    cv2.imshow("Original Crop",              crop)
    cv2.imshow("Yellow Mask (Bun)",          yellow_mask)
    cv2.imshow("Red Mask (Patty)",           red_mask)

    print("Press any key on an image window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()