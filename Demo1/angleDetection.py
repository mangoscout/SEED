import numpy as np
import cv2
import os
import board
from adafruit_character_lcd.character_lcd_rgb_i2c import Character_LCD_RGB_I2C

#setting up camera data,uses calibration coefficients data 
CALIB_FILE = "camera_calib_once.npz"
CAM_INDEX = 0        #same as calibration code         
MARKER_LENGTH_M = 0.1   # 100m on marker sheet
ARUCO_DICT = cv2.aruco.DICT_4X4_50

if not os.path.exists(CALIB_FILE):
    raise FileNotFoundError(f"{CALIB_FILE} not found. Run calibrate_once.py first.")

data = np.load(CALIB_FILE)
K = data["K"]
dist = data["dist"]

# LCD setup
i2c = board.I2C()
lcd = Character_LCD_RGB_I2C(i2c, 16, 2)
lcd.clear()
lcd.message = "Angle:\n--.- deg"

cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Could not open camera.")

# Warm up camera
for _ in range(15):
    cap.read()

ok, frame = cap.read()
if not ok or frame is None:
    raise RuntimeError("Could not read from camera.")

h, w = frame.shape[:2]

# undistortion setup
newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0.0, newImgSize=(w, h))
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_16SC2)

x, y, rw, rh = roi

# camera matrix for the cropped undistorted image
croppedK = newK.copy()
croppedK[0, 2] -= x
croppedK[1, 2] -= y

# Aruco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

# Object points for square marker, centered at (0,0,0)
# Open CV code referened 
# top-left, top-right, bottom-right, bottom-left
L = MARKER_LENGTH_M
obj_points = np.array([
    [-L / 2,  L / 2, 0.0],
    [ L / 2,  L / 2, 0.0],
    [ L / 2, -L / 2, 0.0],
    [-L / 2, -L / 2, 0.0]
], dtype=np.float32)

last_lcd_text = "Angle:\n--.- deg"

print("Live undistorted feed running.")
print("Press 'q' to quit.")

# creating undistored camera 
while True:
    ok, frame = cap.read()
    if not ok or frame is None:
        continue

    # undistort full frame
    undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Crop to valid size
    undist_cropped = undist[y:y+rh, x:x+rw].copy()

    # Detect markers
    corners, ids, _ = detector.detectMarkers(undist_cropped)

    angle_text = "X Angle: --.- deg"
    
    if ids is not None and len(ids) > 0:
        # Use the first detected marker
        img_points = corners[0].reshape(4, 2).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            croppedK,
            None,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if success:
            # Draw axis on live image
            cv2.drawFrameAxes(undist_cropped, croppedK, None, rvec, tvec, MARKER_LENGTH_M * 0.5)

            t = tvec.reshape(3)

            # X angle from camera axis to marker center
            x_angle_deg = np.degrees(np.arctan2(t[0], t[2]))
            angle_text = f"X Angle: {x_angle_deg:.1f} deg"

    # Print only this angle in the corner
    cv2.putText(
        undist_cropped,
        angle_text,
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    # Print the same value to the LCD
    lcd_value = angle_text.split(": ")[1]  
    lcd_text = f"Angle:\n{lcd_value}"

    if lcd_text != last_lcd_text:
        lcd.message = lcd_text
        last_lcd_text = lcd_text


    cv2.imshow("Undistorted (cropped)", undist_cropped)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

lcd.clear()
cap.release()
cv2.destroyAllWindows()

