Final Demo Documentation – Localization and Controls Team Code

Purpose: The purpose of this program is to control a two-wheel robot so it can autonomously locate, align with, and navigate toward a 
visual marker using feedback from a Raspberry Pi. The robot receives real-time marker data in the format (color, turn command, distance, angle) 
through I2C communication. Once powered on, the robot begins by searching for a marker if none is detected. When a marker is found, the robot aligns 
itself to a target angle of approximately 20 degrees using the angle provided by the Pi. After alignment, the robot drives forward toward the marker 
while continuously monitoring the Pi-reported distance, stopping when it reaches approximately 1 meter from the marker with a small tolerance to 
account for sensor noise. If the robot fails to reach the marker within a timeout period, it stops and restarts the cycle without performing a 
turn. When the robot successfully reaches the marker, it performs a turn based on the detected color, where green (1) results in a right turn, 
red (0) results in a left turn, and no color (-1) results in no turn. Encoder counts are used to measure wheel motion for both turning and forward 
movement, improving consistency and repeatability, while additional safeguards prevent stale data from causing incorrect behavior.

Organization: The code is organized into several main sections. First, the program defines hardware configuration, including motor pins, encoder pins, 
and I2C communication settings, along with robot physical parameters such as wheel radius, wheel base, and encoder counts per revolution. These values 
are used to compute accurate movement distances and turning angles. Encoder interrupt service routines are implemented to continuously track wheel rotation, 
allowing the robot to measure motion in real time. Motor helper functions are included to control direction, adjust PWM speeds, and stop the robot, with an 
additional right-wheel PWM boost to compensate for mechanical imbalance and improve straight-line driving. The program also includes an I2C receive function 
that collects and assembles incoming data from the Raspberry Pi, then parses it to extract color, distance, and angle information. These values are updated 
using interrupt-safe methods to ensure reliable readings during motion. The main behavior of the robot is divided into functional steps: a search function 
that rotates the robot until a marker is detected, an alignment function that adjusts orientation to reach a target angle, a forward movement function that 
drives toward the marker and stops based on Pi distance or timeout, and a final turn function that performs a 90-degree turn based on detected color or does 
nothing if no color is detected. Finally, the setup() function initializes all hardware and communication systems and includes a short startup delay, while 
the loop() function repeatedly executes the full navigation cycle of searching, aligning, moving forward, and performing the final turn.
