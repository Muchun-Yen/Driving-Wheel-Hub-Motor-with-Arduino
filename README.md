#Driving Wheel Hub Motor with Arduino

The follows picures shows the wheel hub motor which we used with 6" diameter, 30 poles, and 27 windings.
![The Wheel Hub Motor](https://github.com/Muchun-Yen/Driving-Wheel-Hub-Motor-with-Arduino/blob/master/The%20Wheel%20Hub%20Motor.jpeg)

## In this project the BLDC drive board support by NTHU.
![BLDC Drive Board](https://github.com/Muchun-Yen/Driving-Wheel-Hub-Motor-with-Arduino/blob/master/BLDC%20Drive%20Board.png)

In this system architecture, we need a board with at least 2 serial ports for communication between motor drive board and upper device.
Therefor We choosed the Arduino mega 2560 which has 3 HW serial ports.
one Arduino mega 2560 board works for control the left wheel hub motor. Another is doing the same thing for right wheel hub motor.
Both of the two Arduino mega 2560 boards work independent and always ready to receive control messages from upper device, and feedback the motor control related infomation to it.

the last one Arduino Mega 2560 board works for build a bridge for connect ROS system and motor control system.

## There is the system diagram for reference.
![Arduino Control System Design](https://github.com/Muchun-Yen/Driving-Wheel-Hub-Motor-with-Arduino/blob/master/Arduino%20Control%20System%20Design.png)

## Operation Steps
	After you download the sourcode to the Arduino mega 2560 boards, you can test by the following steps.
```javascript
Terminal 1:$ roscore
Terminal 2:$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
Terminal 3:$ rostopic echo /feedback_wheel_angularVel
Terminal 4:$ rostopic echo /feedback_wheel_voltageQ
```

#### If you want to control the speed you can change x/y values for left/right wheel speed rads/sec
```javascript 
Terminal 5:$ rostopic pub -1 /cmd_wheel_angularVel geometry_msgs/Vector3 "x: 0.0 
y: 0.0 
z: 0.0"
```

###### for example 
```javascript 
Terminal 5:$ rostopic pub -1 /cmd_wheel_angularVel geometry_msgs/Vector3 "x: 2.0 
y: 1.0 
z: 0.0"
```
It set up the left wheel speed to 2 rads/sec, and right wgeel soeed setting to 1 rads/sec.(reserve the z value)

#### If you want to control the Vq you can change x/y values for left/right wheel Vq values, and setting the operation mode to Vq control by z/w values.
```javascript 
Terminal 5:$ rostopic pub -1 /cmd_wheel_voltageQ geometry_msgs/Quaternion "x: 0.0 
y: 0.0 
z: 0.0"
w: 0.0"
```	 

###### for example 
```javascript 
Terminal 5:$ rostopic pub -1 /cmd_wheel_voltageQ geometry_msgs/Quaternion "x: 320 
y: 128 
z: 2"
w: 2"
```	
It set up the left wheel speed to 1 rads/sec, and right wgeel soeed setting to 0.5 rads/sec.
and left control mode switch (z value) to Vq control open loop. and right control mode switch (w value) to Vq control open loop.

#### cmd_wheel_voltageQ geometry_msgs can switch left/right wheel control mode by z/w value.
When z/w value equal 1 it is operating close loop, which control by speed setting and PID control system.
When z/w value equal 2 it is operating open loop, which control by Vq setting and no PID control system.



