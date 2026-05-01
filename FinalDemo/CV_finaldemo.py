import numpy as np
import cv2
import os
import board
import busio
import time
from adafruit_character_lcd.character_lcd_rgb_i2c import Character_LCD_RGB_I2C

CALIB_FILE = "camera_calib_once.npz"
CAM_INDEX = 0
MARKER_LENGTH_M = 0.1016
ARUCO_DICT = cv2.aruco.DICT_6X6_250

# Arrow detection tuning
ARROW_PAD_X = 100
ARROW_PAD_Y = 90
ARROW_MIN_AREA = 250
THRESH_VAL = 120

# Only run heavy vision logic every 3rd frame
PROCESS_EVERY_N_FRAMES = 10

# ---------------- I2C SETUP FOR ARDUINO ----------------
# Common Arduino I2C slave address
ARDUINO_I2C_ADDR = 0x08

# Use bus 1 on Raspberry Pi
try:
    import smbus2
    arduino_bus = smbus2.SMBus(1)
    arduino_available = True
except Exception:
    arduino_bus = None
    arduino_available = False
# -------------------------------------------------------

if not os.path.exists(CALIB_FILE):
    raise FileNotFoundError(f"{CALIB_FILE} not found. Run calibrate_once.py first.")

data = np.load(CALIB_FILE)
K = data["K"]
dist = data["dist"]

# LCD setup (also on I2C)
#i2c = board.I2C()
#lcd = Character_LCD_RGB_I2C(i2c, 16, 2)
#lcd.clear()
#lcd.message = "Angle:\n--.- deg"

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
newK, roi = cv2.getOptimalNewCameraMatrix(
    K, dist, (w, h), alpha=0.0, newImgSize=(w, h)
)
map1, map2 = cv2.initUndistortRectifyMap(
    K, dist, None, newK, (w, h), cv2.CV_16SC2
)
x, y, rw, rh = roi

croppedK = newK.copy()
croppedK[0, 2] -= x
croppedK[1, 2] -= y

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = cv2.aruco.DetectorParameters()
detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
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

# Saved results for skipped frames
frame_count = 0
last_x_angle_deg = None
last_distance_m = None
last_turn_cmd = 0
last_arrow_text = "NONE"
last_arrow_center = None
last_arrow_roi = None
last_marker_for_draw = None
last_marker_points = None
last_ids = None


def detect_arrow_side(frame_bgr, marker_points):
    img_points = marker_points.reshape(4, 2).astype(np.int32)

    marker_xmin = int(np.min(img_points[:, 0]))
    marker_xmax = int(np.max(img_points[:, 0]))
    marker_ymin = int(np.min(img_points[:, 1]))
    marker_ymax = int(np.max(img_points[:, 1]))

    marker_cx = int(np.mean(img_points[:, 0]))

    roi_xmin = max(0, marker_xmin - ARROW_PAD_X)
    roi_xmax = min(frame_bgr.shape[1], marker_xmax + ARROW_PAD_X)
    roi_ymin = max(0, marker_ymin - ARROW_PAD_Y)
    roi_ymax = min(frame_bgr.shape[0], marker_ymax + ARROW_PAD_Y)

    roi = frame_bgr[roi_ymin:roi_ymax, roi_xmin:roi_xmax].copy()

    if roi.size == 0:
        return 0, None, (roi_xmin, roi_ymin, roi_xmax, roi_ymax)

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    _, mask = cv2.threshold(gray, THRESH_VAL, 255, cv2.THRESH_BINARY_INV)

    kernel_open = np.ones((3, 3), np.uint8)
    kernel_close = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

    # Remove the ArUco marker area itself from the mask
    local_marker = img_points.copy()
    local_marker[:, 0] -= roi_xmin
    local_marker[:, 1] -= roi_ymin
    cv2.fillConvexPoly(mask, local_marker, 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < ARROW_MIN_AREA:
            continue

        x_box, y_box, w_box, h_box = cv2.boundingRect(contour)
        if w_box < 10 or h_box < 10:
            continue

        if area > best_area:
            best_area = area
            best_contour = contour

    if best_contour is None:
        return 0, None, (roi_xmin, roi_ymin, roi_xmax, roi_ymax)

    M = cv2.moments(best_contour)
    if M["m00"] == 0:
        return 0, None, (roi_xmin, roi_ymin, roi_xmax, roi_ymax)

    arrow_cx_local = int(M["m10"] / M["m00"])
    arrow_cy_local = int(M["m01"] / M["m00"])

    arrow_cx = roi_xmin + arrow_cx_local
    arrow_cy = roi_ymin + arrow_cy_local

    if arrow_cx < marker_cx:
        turn_cmd = -90
    elif arrow_cx > marker_cx:
        turn_cmd = 90
    else:
        turn_cmd = 0

    return turn_cmd, (arrow_cx, arrow_cy), (roi_xmin, roi_ymin, roi_xmax, roi_ymax)


def send_packet_i2c(packet_str):
    global arduino_bus, arduino_available

    if not arduino_available or arduino_bus is None:
        return

    try:
        # Convert string to list of byte values
        byte_data = [ord(c) for c in packet_str]

        # Send in chunks because I2C block writes are limited
        chunk_size = 16
        for i in range(0, len(byte_data), chunk_size):
            chunk = byte_data[i:i + chunk_size]
            arduino_bus.write_i2c_block_data(ARDUINO_I2C_ADDR, 0, chunk)
            time.sleep(0.01)

    except Exception as e:
        print("I2C send error:", e)


while True:
    ok, frame = cap.read()
    if not ok:
        continue

    undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    undist_cropped = undist[y:y + rh, x:x + rw].copy()

    frame_count += 1

    # Default to last known values so skipped frames still show data
    x_angle_deg = last_x_angle_deg
    distance_m = last_distance_m
    turn_cmd = last_turn_cmd
    arrow_text = last_arrow_text
    arrow_center = last_arrow_center
    arrow_roi = last_arrow_roi
    marker_for_draw = last_marker_for_draw
    marker_points_to_use = last_marker_points
    ids_to_draw = last_ids

    # Only process every 3rd frame
    if frame_count % PROCESS_EVERY_N_FRAMES == 0:
        x_angle_deg = None
        distance_m = None
        turn_cmd = 0
        arrow_text = "NONE"
        arrow_center = None
        arrow_roi = None
        marker_for_draw = None
        marker_points_to_use = None
        ids_to_draw = None

        corners, ids, _ = detector.detectMarkers(undist_cropped)

        if ids is not None:
            for i, marker in enumerate(corners):
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

                    turn_cmd, arrow_center, arrow_roi = detect_arrow_side(
                        undist_cropped,
                        marker.reshape(4, 2)
                    )

                    if turn_cmd == 90:
                        arrow_text = "RIGHT"
                    elif turn_cmd == -90:
                        arrow_text = "LEFT"
                    else:
                        arrow_text = "NONE"

                    marker_for_draw = marker.astype(np.float32)
                    marker_points_to_use = marker.reshape(4, 2).astype(np.int32)
                    ids_to_draw = ids[i:i+1]

                    last_x_angle_deg = x_angle_deg
                    last_distance_m = distance_m
                    last_turn_cmd = turn_cmd
                    last_arrow_text = arrow_text
                    last_arrow_center = arrow_center
                    last_arrow_roi = arrow_roi
                    last_marker_for_draw = marker_for_draw
                    last_marker_points = marker_points_to_use
                    last_ids = ids_to_draw
                    break
            else:
                last_x_angle_deg = None
                last_distance_m = None
                last_turn_cmd = 0
                last_arrow_text = "NONE"
                last_arrow_center = None
                last_arrow_roi = None
                last_marker_for_draw = None
                last_marker_points = None
                last_ids = None
        else:
            last_x_angle_deg = None
            last_distance_m = None
            last_turn_cmd = 0
            last_arrow_text = "NONE"
            last_arrow_center = None
            last_arrow_roi = None
            last_marker_for_draw = None
            last_marker_points = None
            last_ids = None

    # Draw saved marker box
    if marker_for_draw is not None and ids_to_draw is not None:
        cv2.aruco.drawDetectedMarkers(
            undist_cropped,
            [marker_for_draw],
            ids_to_draw,
            borderColor=(255, 0, 255)
        )

    # Draw saved ROI
    if arrow_roi is not None:
        roi_xmin, roi_ymin, roi_xmax, roi_ymax = arrow_roi
        cv2.rectangle(
            undist_cropped,
            (roi_xmin, roi_ymin),
            (roi_xmax, roi_ymax),
            (255, 255, 0),
            2
        )

    # Draw saved marker center and arrow center
    if marker_points_to_use is not None:
        marker_cx = int(np.mean(marker_points_to_use[:, 0]))
        marker_cy = int(np.mean(marker_points_to_use[:, 1]))
        cv2.circle(undist_cropped, (marker_cx, marker_cy), 5, (255, 0, 255), -1)

        if arrow_center is not None:
            cv2.circle(undist_cropped, arrow_center, 5, (0, 255, 255), -1)
            cv2.line(
                undist_cropped,
                (marker_cx, marker_cy),
                arrow_center,
                (0, 255, 255),
                2
            )

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
        f"Arrow: {arrow_text}",
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

  #  if x_angle_deg is not None:
     #   lcd_text = f"Angle:\n{x_angle_deg:.1f} deg"
    #else:
    #    lcd_text = "Angle:\n--.- deg"

    #if lcd_text != last_lcd_text:
        #lcd.clear()
        #lcd.message = lcd_text
        #last_lcd_text = lcd_text

    # Send data to Arduino over I2C
    if distance_m is not None and x_angle_deg is not None:
        packet = f"{turn_cmd},{distance_m:.3f},{x_angle_deg:.1f}\n"
    else:
        packet = "0,-1.000,-999.0\n"

    if packet != last_sent_packet:
        send_packet_i2c(packet)
        print("Sent I2C:", packet.strip())
        last_sent_packet = packet

    cv2.imshow("Vision", undist_cropped)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

lcd.clear()
cap.release()
cv2.destroyAllWindows()

