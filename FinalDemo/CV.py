import numpy as np
import cv2
import os
import board
import serial
import time
from adafruit_character_lcd.character_lcd_rgb_i2c import Character_LCD_RGB_I2C

CALIB_FILE = "camera_calib_once.npz"
CAM_INDEX = 0
MARKER_LENGTH_M = 0.1016
ARUCO_DICT = cv2.aruco.DICT_6X6_250

STOP_CMD = 999

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

# Serial setup
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except:
    arduino = None

# Camera setup
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    raise RuntimeError("Could not open camera.")

for _ in range(15):
    cap.read()

ok, frame = cap.read()
if not ok:
    raise RuntimeError("Could not read from camera.")

h, w = frame.shape[:2]

# Undistortion
newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0.0, newImgSize=(w, h))
map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_16SC2)
x, y, rw, rh = roi

croppedK = newK.copy()
croppedK[0, 2] -= x
croppedK[1, 2] -= y

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = cv2.aruco.DetectorParameters()
detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

L = MARKER_LENGTH_M
obj_points = np.array([
    [-L / 2,  L / 2, 0.0],
    [ L / 2,  L / 2, 0.0],
    [ L / 2, -L / 2, 0.0],
    [-L / 2, -L / 2, 0.0]
], dtype=np.float32)

last_lcd_text = ""
last_sent_packet = ""

# =========================
# NEW: Arrow shape detection
# =========================
def detect_arrow_direction(bgr_img):
    gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return -1, "NONE", 0

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < 200:
        return -1, "NONE", 0

    # Draw contour for debugging
    cv2.drawContours(bgr_img, [largest], -1, (255, 0, 0), 2)

    x, y, w, h = cv2.boundingRect(largest)

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return -1, "NONE", 0

    cx = int(M["m10"] / M["m00"])
    box_center = x + w // 2

    if cx < box_center:
        return 1, "GREEN", -90   # LEFT
    else:
        return 0, "RED", 90      # RIGHT


while True:
    ok, frame = cap.read()
    if not ok:
        continue

    undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    undist_cropped = undist[y:y + rh, x:x + rw].copy()

    x_angle_deg = None
    distance_m = None
    color_code = -1
    color_name = "NONE"
    turn_cmd = 0
    marker_detected = False

    corners, ids, _ = detector.detectMarkers(undist_cropped)

    if ids is not None:
        marker_detected = True

        cv2.aruco.drawDetectedMarkers(undist_cropped, corners, ids, borderColor=(255, 0, 255))

        for marker in corners:
            img_points = marker.reshape(4, 2).astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                croppedK,
                None,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if success:
                t = tvec.reshape(3)

                x_angle_deg = float(np.degrees(np.arctan2(t[0], t[2])))
                distance_m = float(np.linalg.norm(t))

                xmin = int(np.min(img_points[:, 0]))
                xmax = int(np.max(img_points[:, 0]))
                ymin = int(np.min(img_points[:, 1]))
                ymax = int(np.max(img_points[:, 1]))

                pad = 100

                xmin = max(0, xmin - pad)
                xmax = min(rw, xmax + pad)
                ymin = max(0, ymin - pad)
                ymax = min(rh, ymax + pad)

                roi = undist_cropped[ymin:ymax, xmin:xmax]

                color_code, color_name, turn_cmd = detect_arrow_direction(roi)

                cv2.rectangle(undist_cropped, (xmin, ymin), (xmax, ymax), (255, 0, 255), 2)

                break

    # STOP logic
    if color_code == -1:
        if marker_detected:
            turn_cmd = STOP_CMD
        else:
            turn_cmd = 0

    angle_text = "--.-"
    dist_text = "--.-"

    if x_angle_deg is not None:
        angle_text = f"{x_angle_deg:.1f}"

    if distance_m is not None:
        dist_text = f"{distance_m:.2f}"

    cv2.putText(undist_cropped, f"Angle: {angle_text}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.putText(undist_cropped, f"Dist: {dist_text}", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    cv2.putText(undist_cropped, f"Color: {color_name}", (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.putText(undist_cropped, f"Turn Cmd: {turn_cmd}", (20, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 200, 0), 2)

    if x_angle_deg is not None:
        lcd_text = f"Angle:\n{x_angle_deg:.1f} deg"
    else:
        lcd_text = "Angle:\n--.- deg"

    if lcd_text != last_lcd_text:
        lcd.clear()
        lcd.message = lcd_text
        last_lcd_text = lcd_text

    if arduino is not None:
        if distance_m is not None and x_angle_deg is not None:
            packet = f"{color_code},{turn_cmd},{distance_m:.3f},{x_angle_deg:.1f}\n"
        else:
            packet = f"{color_code},0,-1.000,-999.0\n"

        if packet != last_sent_packet:
            arduino.write(packet.encode())
            last_sent_packet = packet

    cv2.imshow("Vision", undist_cropped)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

lcd.clear()
cap.release()
cv2.destroyAllWindows()

if arduino is not None:
    arduino.close()
