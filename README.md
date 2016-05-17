#Driving Wheel Hub Motor with Arduino

The Wheel hub motor which we used with 6" radius, 30 poles, and 27 windings 
![The Wheel Hub Motor](https://github.com/Muchun-Yen/Driving-Wheel-Hub-Motor-with-Arduino/blob/master/The%20Wheel%20Hub%20Motor.jpeg)

In this project the BLDC drive board support by NTHU.
![BLDC Drive Board](https://github.com/Muchun-Yen/Driving-Wheel-Hub-Motor-with-Arduino/blob/master/BLDC%20Drive%20Board.png)

At least 2 serial ports for communication needed between motor drive board and upper device. We choosed the Arduino mega 2560 which has 3 HW serial ports.

one Arduino mega 2560 board works for control the left wheel hub motor. Another Arduino mega 2560 is doing the same thing for right wheel hub motor.
The two Arduino mega boards works independent and always ready to receive control messages from upper device, and feedback the motor control related infomation to it.

the last one Arduino Mega 2560 board works for build a bridge for connect ROS system and motor control system.

There is a system diagram for reference.
![Arduino Control System Design](https://github.com/Muchun-Yen/Driving-Wheel-Hub-Motor-with-Arduino/blob/master/Arduino%20Control%20System%20Design.png)
