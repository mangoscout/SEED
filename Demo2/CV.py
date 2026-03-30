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

# Hold last good color result briefly so detection does not flicker
last_color_code = -1
last_color_name = "NONE"
last_turn_cmd = 0
last_color_seen_frames = 0
COLOR_HOLD_FRAMES = 5


def get_largest_blob(mask, bgr_img, box_color, min_area=180):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return 0

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < min_area:
        return 0

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2
    cy = y + h // 2

    cv2.rectangle(bgr_img, (x, y), (x + w, y + h), box_color, 2)
    cv2.circle(bgr_img, (cx, cy), 5, box_color, -1)

    return area


def detect_arrow_color(bgr_img):
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # RED masks
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # GREEN mask
    lower_green = np.array([35, 60, 60])
    upper_green = np.array([95, 255, 255])

    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel_open = np.ones((3, 3), np.uint8)
    kernel_close = np.ones((5, 5), np.uint8)
    kernel_erode = np.ones((3, 3), np.uint8)

    # Clean red mask
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_open)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)

    # Clean green mask
    green_mask = cv2.erode(green_mask, kernel_erode, iterations=2)
    green_mask = cv2.dilate(green_mask, kernel_erode, iterations=2)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel_close)

    red_area = get_largest_blob(red_mask, bgr_img, (0, 0, 255), min_area=180)
    green_area = get_largest_blob(green_mask, bgr_img, (0, 255, 0), min_area=180)

    color_code = -1
    color_name = "NONE"
    turn_cmd = 0

    # GREEN = LEFT = -90
    # RED   = RIGHT = 90
    if green_area > 0 and green_area >= red_area:
        color_code = 1
        color_name = "GREEN"
        turn_cmd = -90
    elif red_area > 0:
        color_code = 0
        color_name = "RED"
        turn_cmd = 90

    return color_code, color_name, turn_cmd


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

    corners, ids, _ = detector.detectMarkers(undist_cropped)

    if ids is not None:
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

                pad = 150

                xmin = max(0, xmin - pad)
                xmax = min(rw, xmax + pad)
                ymin = max(0, ymin - pad)
                ymax = min(rh, ymax + pad)

                roi = undist_cropped[ymin:ymax, xmin:xmax]

                color_code, color_name, turn_cmd = detect_arrow_color(roi)

                cv2.rectangle(undist_cropped, (xmin, ymin), (xmax, ymax), (255, 0, 255), 2)

                break

    # Hold last good color briefly to prevent flicker when camera moves
    if color_code != -1:
        last_color_code = color_code
        last_color_name = color_name
        last_turn_cmd = turn_cmd
        last_color_seen_frames = COLOR_HOLD_FRAMES
    elif last_color_seen_frames > 0:
        color_code = last_color_code
        color_name = last_color_name
        turn_cmd = last_turn_cmd
        last_color_seen_frames -= 1
    else:
        color_code = -1
        color_name = "NONE"
        turn_cmd = 0

    angle_text = "--.-"
    dist_text = "--.-"

    if x_angle_deg is not None:
        angle_text = f"{x_angle_deg:.1f}"

    if distance_m is not None:
        dist_text = f"{distance_m:.2f}"

    cv2.putText(
        undist_cropped,
        f"Angle: {angle_text}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

    cv2.putText(
        undist_cropped,
        f"Dist: {dist_text}",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 0),
        2
    )

    cv2.putText(
        undist_cropped,
        f"Color: {color_name}",
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        undist_cropped,
        f"Turn Cmd: {turn_cmd}",
        (20, 160),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 200, 0),
        2
    )

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

