import cv2
import numpy as np


# Change this line to import directly from the neighboring file
from vision.detection_burger_pieces import classify_burger_piece, BUN_YELLOW_LOW, BUN_YELLOW_HIGH

def mouse_callback(event, x, y, flags, param):
    """Prints HSV values when you click on the image."""
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_img = param['hsv']
        h, s, v = hsv_img[y, x]
        print(f"📍 Clicked Pixel (X: {x}, Y: {y}) -> HSV: ({h}, {s}, {v})")
        print(f"   💡 Suggested Tuning Range: Low({max(0, h-15)}, {max(0, s-40)}, {max(0, v-40)}) to High({min(180, h+15)}, 255, 255)\n")

def main():
    # 1. Load a test image crop of a bun or patty
    image_path = "IMG_6955.jpg"  # Replace with your test image path
    crop = cv2.imread(image_path)
    
    if crop is None:
        print(f"❌ Error: Could not load image from '{image_path}'. Check the path!")
        return

    # 2. Run your existing API
    result = classify_burger_piece(crop)
    print("═" * 40)
    print("  CLASSIFICATION RESULTS")
    print("═" * 40)
    for k, v in result.items():
        print(f"{k:12}: {v}")
    print("═" * 40)
    print("👉 CLICK on the 'Original Crop' window to print local HSV values.\n")

    # 3. Recreate HSV for the mouse callback tool
    hsv = cv2.cvtColor(cv2.GaussianBlur(crop, (5, 5), 0), cv2.COLOR_BGR2HSV)
    
    # Create windows
    cv2.namedWindow("Original Crop")
    cv2.setMouseCallback("Original Crop", mouse_callback, param={'hsv': hsv})

    # 4. Debug visualizer: Let's see what the yellow mask looks like live
    # (You can swap this with the red mask logic to debug patties)
    yellow_mask = cv2.inRange(hsv, BUN_YELLOW_LOW, BUN_YELLOW_HIGH)
    
    # Display everything
    cv2.imshow("Original Crop", crop)
    cv2.imshow("Yellow Mask (Before Morphology)", yellow_mask)
    
    print("Press any key on an image window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()