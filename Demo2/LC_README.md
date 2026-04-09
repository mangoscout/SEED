Demo 2 Documentation – Localization and Controls Team Code

Purpose:
The purpose of this program is to control a two-wheel robot so it can autonomously locate, align with, and navigate toward a visual marker using feedback from a Raspberry Pi. The robot receives real-time marker data (color, distance, and angle) through I2C communication.
Once powered on, the robot begins searching for a marker by rotating in small increments. When a marker is detected, the robot aligns itself to a target angle range using the angle provided by the Pi. After alignment, the robot drives forward a preset distance using encoder feedback while also monitoring the Pi-reported distance to stop early if it gets close enough to the marker. Finally, the robot performs a 90-degree turn based on the detected marker color (red = right turn, green = left turn).
Encoder counts are used to measure wheel motion for both turning and forward movement, ensuring accurate and repeatable motion control.

Organization:
The code is organized into several main sections. First, the program defines the motor pins, encoder pins, I2C settings, and robot physical parameters such as wheel radius, wheel base, and encoder counts per revolution. It also includes configuration values for movement speeds, alignment thresholds, and stopping conditions.
Next, encoder interrupt service routines are used to continuously track wheel rotation during motion. Motor helper functions are included to control motor direction and speed, as well as to stop the robot.
The program also includes an I2C receive function that parses incoming data from the Raspberry Pi, updating the robot with the current marker distance, angle, and color.

The main behavior of the robot is divided into several functional steps:
 - A search function that rotates the robot until a marker is detected
 - An alignment function that adjusts the robot’s orientation to fall within a target angle range
 - A forward movement function that drives the robot a specified distance using encoder feedback and stops early if the Pi-reported distance threshold is reached
 - A final turn function that rotates the robot 90 degrees based on marker color
Finally, the setup() function initializes all hardware components and communication, while the loop() function executes the full sequence of searching, aligning, moving forward, and performing the final turn.
