# Demo 2 Code
CV.py= Computer Vision team code

Purpose: The purpose of CV.py is to detect an ArUco marker in a live camera feed, estimate its position relative to the camera, and determine both the horizontal angle and distance to the marker. The program uses precomputed camera calibration parameters to undistort images for improved accuracy. In addition to angle and distance estimation, the program detects the color of an arrow near the marker and determines a corresponding turn command (left or right). The computed information is displayed on a live video feed, sent to an LCD screen, and transmitted to an Arduino via serial communication for robot control.

Organization: The file loads camera calibration data, initializes the camera, LCD, and Arduino connection, and undistorts incoming frames. It then detects ArUco markers, estimates their position to compute angle and distance, and analyzes a region around the marker to detect arrow color and determine a turn command. The results are displayed on the video feed and LCD, and sent to the Arduino, with a short memory of previous detections to reduce flickering.
