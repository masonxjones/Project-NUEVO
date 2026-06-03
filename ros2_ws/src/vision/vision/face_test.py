import cv2
import time
import os

# Configuration
CAMERA_DEVICE = "/dev/video10"
MATCH_SIZE = (128, 128)
THRESHOLD = 0.40  # Minimum confidence score to claim a match

# ABSOLUTE PATHS: Guarantees Python finds your images from anywhere
PATH_A = "/home/bd911/Project-NUEVO/ros2_ws/src/vision/data/customer_a.jpeg"
PATH_B = "/home/bd911/Project-NUEVO/ros2_ws/src/vision/data/customer_b.jpeg"
PATH_PREVIEW = "/home/bd911/Project-NUEVO/ros2_ws/src/vision/data/latest.jpg"

def load_and_train_pattern(image_path, label):
    """Loads a pre-existing JPEG image and processes it into a normalized matching pattern."""
    if not os.path.exists(image_path):
        print(f"[ERROR] Reference file missing for Customer {label} at: {image_path}")
        return None
        
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] Could not read image at {image_path}. Check file integrity.")
        return None
        
    # Standardize size and drop color channels to focus purely on structural features
    resized = cv2.resize(img, MATCH_SIZE)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    print(f"[TRAIN] Template matrix successfully initialized for Customer {label}")
    return gray

def main():
    print("==================================================")
    print("       PHASE 1: INITIALIZING TRAINED TEMPLATES    ")
    print("==================================================")
    
    pattern_a = load_and_train_pattern(PATH_A, "A")
    pattern_b = load_and_train_pattern(PATH_B, "B")
    
    if pattern_a is None or pattern_b is None:
        print("\n[CRITICAL] System setup failed: Missing reference images. Exiting.")
        return

    print("\nInitializing camera hardware stream...")
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        print(f"Failed to access {CAMERA_DEVICE}. Trying fallback device /dev/video0...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[CRITICAL] No active video device detected.")
            return

    print("\n==================================================")
    print("       PHASE 2: LIVE CONFIDENCE MATCHING STREAM   ")
    print("==================================================")
    print(f"[LIVE] Diagnostic stream active. Viewing box saved to: {PATH_PREVIEW}")
    print("[LIVE] Processing frames... Press Ctrl+C to stop.\n")
    time.sleep(1)

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # Calculate dimensions for center cropping
            h, w = frame.shape[:2]
            size = min(h, w)
            cy, cx = h // 2, w // 2
            
            # --- DIAGNOSTIC PREVIEW GENERATION ---
            # Create a user visual reference showing exactly what area is targeted
            preview_frame = frame.copy()
            top_left = (cx - size // 2, cy - size // 2)
            bottom_right = (cx + size // 2, cy + size // 2)
            cv2.rectangle(preview_frame, top_left, bottom_right, (0, 255, 0), 2)
            cv2.imwrite(PATH_PREVIEW, preview_frame)
            # --------------------------------------

            # Extract the raw center square from the camera matrix
            crop = frame[cy - size // 2 : cy + size // 2, cx - size // 2 : cx + size // 2]

            # Convert the live crop matrix to match the training properties
            live_resized = cv2.resize(crop, MATCH_SIZE)
            live_gray = cv2.cvtColor(live_resized, cv2.COLOR_BGR2GRAY)

            # Compute Normalized Cross-Correlation coefficients (-1.0 to 1.0)
            res_a = cv2.matchTemplate(pattern_a, live_gray, cv2.TM_CCOEFF_NORMED)
            res_b = cv2.matchTemplate(pattern_b, live_gray, cv2.TM_CCOEFF_NORMED)

            score_a = float(res_a[0][0])
            score_b = float(res_b[0][0])

            # Identity arbitration based on maximum confidence vs acceptable threshold
            if score_a > score_b and score_a > THRESHOLD:
                print(f"customer A      (Confidence: {score_a:.2f})")
            elif score_b > score_a and score_b > THRESHOLD:
                print(f"customer B      (Confidence: {score_b:.2f})")
            else:
                highest_score = max(score_a, score_b)
                print(f"unknown customer (Highest Confidence: {highest_score:.2f})")

            # Output spacing rate control
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n[INFO] Core process interrupted. Releasing pipeline assets.")
    finally:
        cap.release()

if __name__ == "__main__":
    main()
EOF
