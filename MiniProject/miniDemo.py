##########################################################################################################
## Name: Jessica Batson (camera detection and arduino communication) & Julia Bickley (LCD screen) 
## Date: 2/27/2026
## Description: This code operates the camera and detects the quadrant the aruco marker is in.
##              The arduino and pi are communicating through I2C. When the camera is opened, you
##              should see 4 quardants seperated by green vertical/horizontal lines. when the marker
##              is in a quadrant, it prints to the camera screen what orientation the wheels should
##              be and also sends this logic to the arduino. This code also prints the position of
##              the wheels to the LCD screen and updates every time the position changes.
## How to run:  In the terminal, run "python miniDemo.py" and the camera should open. Make sure that
##              the supporting arduino is uploaded. Once the camera is on, place an aruco marker in any
##              of the quadrants and the camera should detect the orientation/location of the wheels. 
##########################################################################################################

import os
import cv2
import numpy as np
import serial
import time
import threading
import queue
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


# i2c communication 
from smbus2 import SMBus


# fix Qt/Wayland issues (i was having issues getting the camera on)
os.environ["QT_QPA_PLATFORM"] = "xcb"

# sets up camera prameters (length, height, and frame) 
CAM_INDEX = 0
WIDTH, HEIGHT = 640, 480
WARMUP_FRAMES = 30

# reads aruco marker (4x4 1D0)
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# i2c communication to arduino (matches arduino code) 
I2C_BUS_NUM = 1 # pi bus      
I2C_ADDR = 0x08 # arduino bus   


# camera quadrants (direction converts to wheel orientation) 
WHEEL_BITS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}

# power LCD screen 
lcd_queue = queue.Queue()

def lcd_thread():
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)

    lcd.clear()
    lcd.message = "Screen ON\n"

    while True:
        if not lcd_queue.empty():
            lw, rw = lcd_queue.get()
            lcd.clear()
            lcd.message = f"Goal Position:\n{lw} {rw}"

lcdThread = threading.Thread(target=lcd_thread)
lcdThread.daemon = True
lcdThread.start()

# determines what direction the robot should go based on quadrant
# more for printing/ testing help
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
    # opens camera
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # warm up camera
    for _ in range(WARMUP_FRAMES):
        cap.read()

    # I2C bus open
    bus = SMBus(I2C_BUS_NUM)
    print(f"I2C ready: bus {I2C_BUS_NUM}")
    print(f" I2C ready: addr {hex(I2C_ADDR)}")
    
    # aruco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()

    last_cmd = ""

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        h, w = frame.shape[:2]

        # draw quadrant green lines
        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 0), 2)

        # detect aruco marker
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=params)

        cmd = None
        quad = None

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            pts = corners[0][0].astype(int)

            # detect aruco marker center
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            # determine aruco marker quadrant
            quad = quadrant_compass(cx, cy, w, h)

            # converts quadrant to left wheel/ right wheel 
            lw, rw = WHEEL_BITS[quad]
            cmd = f"{lw}{rw}\n"

            # prints on camera what quadrant and wheel orientation the wheels should be
            # more for testing purposes so that the logic is correct 
            marker_id = int(ids.flatten()[0])
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            cv2.putText(frame, f"ID {marker_id} {quad} -> {lw}{rw}",
                        (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # if aruco marker location changes, send command
        if cmd is not None and cmd != last_cmd:
            lw = int(cmd[0])
            rw = int(cmd[1])

            try:
                # prints to terminal the data that is being sent to the arduino
                # for testing logic and testing camera without arduno 
                bus.write_i2c_block_data(I2C_ADDR, 0x01, [lw, rw])
                print(f"I2C is at -> {lw} {rw} in quadrant : {quad}")
            except Exception as e:
                print(f"I2C send failed: {e}")

             # update LCD screen 
            lcd_queue.put((lw, rw))

            last_cmd = cmd

        cv2.imshow("Aruco Detection", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    try:
        bus.close()
    except:
        pass

    if ser is not None:
        ser.close()

if __name__ == "__main__":
    main()
