# Final Demo Code
CV_finaldemo.py= Computer Vision team code

Purpose: The purpose of CV_finaldemo.py is to detect an ArUco marker in a live camera feed, estimate its position relative to the camera, and compute the horizontal angle and distance to the marker. The program also detects an arrow near the marker to determine a turn command (left, right, or none). This information is displayed on the video feed and sent to an Arduino via I2C for robot control. To improve performance and stability, the system processes only every Nth frame and reuses recent detection results between frames.

Organization: The file loads camera calibration data, initializes the camera, LCD, and Arduino connection, and undistorts incoming frames. It then detects ArUco markers, estimates their position to compute angle and distance, and analyzes a region around the marker to detect arrow color and determine a turn command. The results are displayed on the video feed and LCD, and sent to the Arduino, with a short memory of previous detections to reduce flickering.
