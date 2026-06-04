"""
vision/detect_burger_pieces.py
───────────────────────────────
Colour-based burger piece classifier.

Detects patty (red) and bun (yellow) from a cropped image using HSV
colour segmentation — the same approach as traffic_light.py.

Usage in vision_node.py
───────────────────────
    from vision.detect_burger_pieces import classify_burger_piece

    for detection in yolo_detections:
        crop = frame[detection.y:detection.y+detection.height,
                     detection.x:detection.x+detection.width]

        result = classify_burger_piece(crop)
        detection.add_attribute("burger_type",  result["label"],  result["score"])
        detection.add_attribute("patty_score",  str(result["patty_score"]),  result["patty_score"])
        detection.add_attribute("bun_score",    str(result["bun_score"]),    result["bun_score"])

Return value
────────────
    {
        "label":       "patty" | "bun" | "none",
        "score":       float 0–1,   # confidence of the winning label
        "patty_score": float 0–1,   # raw red blob score
        "bun_score":   float 0–1,   # raw yellow blob score
    }

Tuning
──────
    Adjust the HSV ranges at the top of the file under TUNABLE CONSTANTS.
    Use a test image and print the HSV values of the piece centre pixels:

        import cv2
        img = cv2.imread("piece.jpg")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        print(hsv[h//2, w//2])   # centre pixel H, S, V

    Then set the ranges to bracket those values with ±10–20 margin.
"""

from __future__ import annotations

import cv2
import numpy as np


# ════════════════════════════════════════════════════════════════════════════
# TUNABLE CONSTANTS
# ════════════════════════════════════════════════════════════════════════════

# ── Patty (red) HSV ranges ────────────────────────────────────────────────
# Red wraps around 0°/180° in HSV, so two ranges are needed.
# Reused from traffic_light.py — already tested under lab lighting.
PATTY_RED_LOW_1  = (0,   80,  80)
PATTY_RED_HIGH_1 = (12, 255, 255)
PATTY_RED_LOW_2  = (168,  80,  80)
PATTY_RED_HIGH_2 = (180, 255, 255)

# ── Bun (yellow) HSV ranges ───────────────────────────────────────────────
# Yellow sits around H=20–35 in OpenCV HSV (0–180 scale).
# Based on detect_yellow_block — tune S/V floors for your lighting.
BUN_YELLOW_LOW  = (18,  180,  150)
BUN_YELLOW_HIGH = (38, 255, 255)

# ── Morphology kernel ─────────────────────────────────────────────────────
MORPH_KERNEL_SIZE = 5    # pixels — larger removes more noise but loses detail

# ── Decision thresholds ───────────────────────────────────────────────────
MIN_BLOB_RATIO   = 0.02  # fraction of crop area — blobs smaller than this are noise
DOMINANCE_RATIO  = 1.25  # winning score must be this many times larger than runner-up


# ════════════════════════════════════════════════════════════════════════════
# INTERNAL HELPERS
# ════════════════════════════════════════════════════════════════════════════

def _largest_blob_score(mask: np.ndarray, image_area: float) -> float:
    """Return the area of the largest contour as a fraction of image_area."""
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return 0.0
    largest_area = max(cv2.contourArea(c) for c in contours)
    return float(largest_area / image_area)


def _apply_morphology(mask: np.ndarray, kernel) -> np.ndarray:
    """Open then close to remove noise and fill small holes."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


# ════════════════════════════════════════════════════════════════════════════
# PUBLIC API
# ════════════════════════════════════════════════════════════════════════════

def classify_burger_pieces(crop: np.ndarray) -> dict:
    """
    Classify a cropped image as patty, bun, or none.

    Args:
        crop: BGR image crop (numpy array). Typically the bounding-box
              region from the camera frame.

    Returns:
        {
            "label":       "patty" | "bun" | "none",
            "score":       float,   # confidence of winning label (0–1)
            "patty_score": float,   # raw red blob fraction (0–1)
            "bun_score":   float,   # raw yellow blob fraction (0–1)
        }
    """
    empty_result = {
        "label": "none", "score": 0.0,
        "patty_score": 0.0, "bun_score": 0.0,
    }

    if crop is None or crop.size == 0:
        return empty_result

    image_area = float(max(1, crop.shape[0] * crop.shape[1]))

    # ── Pre-process ───────────────────────────────────────────────────────
    blurred = cv2.GaussianBlur(crop, (5, 5), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    kernel  = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
    )

    # ── Patty mask (red, two ranges) ──────────────────────────────────────
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, PATTY_RED_LOW_1,  PATTY_RED_HIGH_1),
        cv2.inRange(hsv, PATTY_RED_LOW_2,  PATTY_RED_HIGH_2),
    )
    red_mask    = _apply_morphology(red_mask, kernel)
    patty_score = _largest_blob_score(red_mask, image_area)

    # ── Bun mask (yellow) ─────────────────────────────────────────────────
    yellow_mask = cv2.inRange(hsv, BUN_YELLOW_LOW, BUN_YELLOW_HIGH)
    yellow_mask = _apply_morphology(yellow_mask, kernel)
    bun_score   = _largest_blob_score(yellow_mask, image_area)

    # ── Decision logic (mirrors traffic_light.py) ─────────────────────────
    if patty_score < MIN_BLOB_RATIO and bun_score < MIN_BLOB_RATIO:
        return {**empty_result,
                "patty_score": patty_score, "bun_score": bun_score}

    if patty_score >= bun_score * DOMINANCE_RATIO:
        return {
            "label": "patty", "score": min(1.0, patty_score),
            "patty_score": patty_score, "bun_score": bun_score,
        }

    if bun_score >= patty_score * DOMINANCE_RATIO:
        return {
            "label": "bun", "score": min(1.0, bun_score),
            "patty_score": patty_score, "bun_score": bun_score,
        }

    # Ambiguous — both colours present but neither dominates
    return {
        "label": "none", "score": min(1.0, max(patty_score, bun_score)),
        "patty_score": patty_score, "bun_score": bun_score,
    }
