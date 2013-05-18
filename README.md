EEBug (Line Follower Robot)  Group Design Project
=================================================

This is the source file of our first year group design project: **A line following** robot.

Code is written for the **Texas Instrument's MSP430 series**. We used the **MSP430G2553** chip.

Features
--------
- Follows a figure-of-8 track using **Proportional-Integral-Derivative (PID) Control Algorithm**
- Waits for 8 seconds before moving as per design specifications. The countdown is displayed on a 7-segment display
- Counts the number of laps and shows the robot's **live position** on the track using the 7-segment display ("Pseudo-GPS")
