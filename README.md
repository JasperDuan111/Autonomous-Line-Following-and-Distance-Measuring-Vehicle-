## Autonomous Line-Following and Distance-Measuring Vehicle​ Based on Arduino
This project aims to design and implement an Arduino-based smart car system capable of performing the following core functions:

Closed-Loop Motor Speed Control:​​
- Utilize encoder feedback for real-time wheel speed measurement.
- Apply a ​​PI algorithm​​ to dynamically adjust motor PWM signals.
- Achieve ​​stable and precise speed control​​ across varying road conditions.
  
Line Following:​​ 
- Deploy a ​​quad-channel grayscale sensor array​​ to detect ground path lines.
- Calculate trajectory deviation using a ​​positional PD control algorithm​​.
- Dynamically adjust left/right motor speeds for ​​autonomous path tracking​​.

Distance Measurement:​​ 
- Integrate the ​​HC-SR04 ultrasonic sensor​​ for real-time obstacle detection.
- Enhance accuracy through ​​multi-sampling and median-filtering algorithms​​.
