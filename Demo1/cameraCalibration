import os
import time
import numpy as np
import cv2

# links to angle code, has to be the same 
CAM_INDEX = 0

# size of board 
PATTERN_SIZE = (8, 5)

# number of frames and how frequently captures are being taken 
TARGET_CAPTURES = 20
MIN_TIME_BETWEEN_CAPTURES_SEC = 0.15
MIN_MEAN_CORNER_MOVE_PX = 2.0

CALIB_FILE = "camera_calib_once.npz"

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

def sharpness_score(bgr_img: np.ndarray) -> float:
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

# object point evaluation 
objp = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []


# open camera 
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("camera broken :(.")

last_accept_time = 0.0
last_mean_corner = None
image_size = None

best_img = None
best_score = -1.0

# print to terminal 
print("calibrating")
print("capturing frames")

# capturing chessboard when in frame 
while len(objpoints) < TARGET_CAPTURES:
    ok, img = cap.read()
    if not ok or img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if image_size is None:
        image_size = (gray.shape[1], gray.shape[0])

    found = False
    corners = None

    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, PATTERN_SIZE, None)

    if not found:
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, flags)

    vis = img.copy()

# eding code once all frames are saved
    if found and corners is not None:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(vis, PATTERN_SIZE, corners2, True)

        now = time.time()
        mean_corner = corners2.reshape(-1, 2).mean(axis=0)

        moved_enough = True
        if last_mean_corner is not None:
            moved_enough = np.linalg.norm(mean_corner - last_mean_corner) >= MIN_MEAN_CORNER_MOVE_PX

        time_ok = (now - last_accept_time) >= MIN_TIME_BETWEEN_CAPTURES_SEC

        if moved_enough and time_ok:
            objpoints.append(objp.copy())
            imgpoints.append(corners2.copy())
            last_accept_time = now
            last_mean_corner = mean_corner

            score = sharpness_score(img)
            if score > best_score:
                best_score = score
                best_img = img.copy()

            print(f"Captured {len(objpoints)}/{TARGET_CAPTURES} | sharpness={score:.1f} | best={best_score:.1f}")

    cv2.putText(
        vis,
        f"Found={found} | Captured {len(objpoints)}/{TARGET_CAPTURES}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0) if found else (0, 0, 255),
        2
    )
    cv2.imshow("Calibration Capture", vis)

    if (cv2.waitKey(1) & 0xFF) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 10:
    raise RuntimeError(f"Too few captures ({len(objpoints)}). Aim for ~10–30.")

# evaluating calibration coefficients/values needed for undistorted camera 
rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

print("\Results: ")
print("Calibration RMS reprojection error:", rms)
print("Camera matrix K:\n", K)
print("Distortion coefficients:\n", dist.ravel())

# saving output to angle code 
np.savez(CALIB_FILE,
         K=K,
         dist=dist,
         image_size=np.array(image_size, dtype=np.int32),
         rms=np.array(float(rms), dtype=np.float64))
print(f"Saved calibration to: {CALIB_FILE}")

