import os
import cv2
import numpy as np
import serial
import time

# Fix Qt/Wayland issues 
os.environ["QT_QPA_PLATFORM"] = "xcb"

# ---------- Settings ----------
CAM_INDEX = 0
WIDTH, HEIGHT = 640, 480
WARMUP_FRAMES = 30

ARUCO_DICT = cv2.aruco.DICT_4X4_50

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 9600
# -----------------------------

# Quadrants
WHEEL_BITS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}

def quadrant_compass(cx, cy, w, h):
    left = cx < (w // 2)
    top = cy < (h // 2)

    if top and left:
        return "NW"
    if top and not left:
        return "NE"
    if not top and left:
        return "SW"
    return "SE"

def main():

    # Open camera
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # Warm up camera
    for _ in range(WARMUP_FRAMES):
        cap.read()

    # Try to connect to Arduino
    ser = None
    serial_connected = False
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        time.sleep(2.0)
        serial_connected = True
        print(f" Arduino connected on {SERIAL_PORT}")
    except:
        print("Arduino not detected (printing commands to terminal)")

    # ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()

    last_cmd = ""

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        h, w = frame.shape[:2]

        # Draw quadrant divider lines
        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 0), 2)

        # Detect marker
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=params)

        cmd = None
        quad = None

        if ids is not None and len(ids) > 0:

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            pts = corners[0][0].astype(int)

            # Marker center
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            # Determine quadrant
            quad = quadrant_compass(cx, cy, w, h)

            # Convert to wheel command for sending to arduino
            lw, rw = WHEEL_BITS[quad]
            cmd = f"{lw}{rw}\n"

            # Draw marker center on camera output
            marker_id = int(ids.flatten()[0])
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            cv2.putText(frame, f"ID {marker_id} {quad} -> {lw}{rw}",
                        (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)

        # Send command
        if cmd is not None and cmd != last_cmd:
            if serial_connected:
                ser.write(cmd.encode("ascii"))
            else:
                print(f"SEND -> {cmd.strip()}   (Marker in {quad})")

            last_cmd = cmd

        cv2.imshow("Aruco Detection", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()

if __name__ == "__main__":
    main()
