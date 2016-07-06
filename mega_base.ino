/*************************************************************
[Version]
20160701
[HW Arduino Mega 2560]
Serial port (Default serial for Connect ROSSerial )
Serial1 port (connect to Motor control board Right wheel)
Serial2 port (connect to Motor control board Left wheel)
Serial3 port (connect to BT (Test only))
*************************************************************/

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>

char val;
char commandArray_L[3];
byte sT_L = 0;  //send start byte
byte sH_L = 0;  //send high byte
byte sL_L = 0;  //send low byte
byte sP_L = 0;  //send stop byte

byte rT_L = 0;  //receive start byte
byte rH_L = 0;  //receive high byte
byte rL_L = 0;  //receive low byte
byte rP_L = 0;  //receive stop byte

char commandArray_R[3];
byte sT_R = 0;  //send start byte
byte sH_R = 0;  //send high byte
byte sL_R = 0;  //send low byte
byte sP_R = 0;  //send stop byte

byte rT_R = 0;  //receive start byte
byte rH_R = 0;  //receive high byte
byte rL_R = 0;  //receive low byte
byte rP_R = 0;  //receive stop byte

#define LOOPTIME 100

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
long dT = 0;
int left_actual_receive = 0;
int left_target_send = 0;
int right_actual_receive = 0;
int right_target_send = 0;

ros::NodeHandle nh;

geometry_msgs::Vector3 vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);

void messageCb(const geometry_msgs::Vector3& msg){
  omega_left_target = msg.x;
  omega_right_target = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> s("cmd_wheel_angularVel", messageCb);

void setup(){
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);


  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
}

void loop(){
  if (Serial3.available()){
    val = Serial3.read();

    if (val == 'm'){
      Serial1.write("m", 1);
      Serial2.write("m", 1);
      Serial3.write("m", 1);
    }
    else if (val == 'k'){
      Serial1.write("k", 1);
      Serial2.write("k", 1);
      Serial3.write("k", 1);
    }
    
  }

  readFeadback_angularVel_L();
  readFeadback_angularVel_R();

  if ((millis() - lastMilli) >= LOOPTIME){
    dT = millis() - lastMilli;
    lastMilli = millis();

    sendCmd_wheel_angularVel_L();
    sendCmd_wheel_angularVel_R();

    vel_msg.x = omega_left_actual;
    vel_msg.y = omega_right_actual;
    p.publish(&vel_msg);
  }
  nh.spinOnce();
}

void readFeadback_angularVel_L(){
  if (Serial2.available() >= 4){
    char rT_L = (char)Serial2.read();
    if (rT_L == '{'){
      char commandArray_L[3];
      Serial2.readBytes(commandArray_L,3);
      byte rH_L = commandArray_L[0];
      byte rL_L = commandArray_L[1];
      char rP_L = commandArray_L[2];
      if (rP_L == '}'){
        left_actual_receive = (rH_L << 8) + rL_L;
//程е2 rev/sec
//        omega_left_actual = double (left_actual_receive * 0.00038349559007538);   //convert received 16 bit integer to actual speed 12.566/32767=3.834955900753807e-4=0.00038349559007538
//程е1 rev/sec        
        omega_left_actual = double (left_actual_receive * 0.00019174779503769);   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

      }
    }

  }
}

void readFeadback_angularVel_R(){
  if (Serial1.available() >= 4){
    char rT_R = (char)Serial1.read();
    if (rT_R == '{'){
      char commandArray_R[3];
      Serial1.readBytes(commandArray_R, 3);
      byte rH_R = commandArray_R[0];
      byte rL_R = commandArray_R[1];
      char rP_R = commandArray_R[2];
      if (rP_R == '}'){
        right_actual_receive = (rH_R << 8) + rL_R;
//程е2 rev/sec        
//        omega_right_actual = double (right_actual_receive * 0.00038349559007538);   //convert received 16 bit integer to actual speed 12.566/32767=3.834955900753807e-4=0.00038349559007538
//程е1 rev/sec 
        omega_right_actual = double (right_actual_receive * 0.00019174779503769);   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
        
      }
    }

  }
}


void sendCmd_wheel_angularVel_L(){
//程е2 rev/sec        
//  if(omega_left_target>12.566) omega_left_target=12.566;
//  else if(omega_left_target<-12.566) omega_left_target=-12.566;
//  left_target_send = int(omega_left_target / 0.00038349559007538);   //convert received 16 bit integer to actual speed 12.566/32767=3.834955900753807e-4=0.00038349559007538
//程е1 rev/sec        
//  if(omega_left_target>6.283) omega_left_target=6.283;
//  else if(omega_left_target<-6.283) omega_left_target=-6.283;
//  left_target_send = int(omega_left_target / 0.00019174779503769);   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

  if(omega_left_target>6.283) omega_left_target=6.283;
  else if(omega_left_target<-6.283) omega_left_target=-6.283;
  left_target_send = int(omega_left_target / 0.00019174779503769);   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

  
  char sT_L = '{'; //send start byte
  byte sH_L = highByte(left_target_send);
  byte sL_L = lowByte(left_target_send);
  char sP_L = '}';
  Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_angularVel_R(){
//程е2 rev/sec    
//  if(omega_right_target>12.566) omega_right_target=12.566;
//  else if(omega_right_target<-12.566) omega_right_target=-12.566;
//  right_target_send = int(omega_right_target / 0.00038349559007538);   //convert received 16 bit integer to actual speed 12.566/32767=3.834955900753807e-4=0.00038349559007538
//程е1 rev/sec    
//  if(omega_right_target>6.283) omega_right_target=6.283;
//  else if(omega_right_target<-6.283) omega_right_target=-6.283;
//  right_target_send = int(omega_right_target / 0.00019174779503769);   //convert received 16 bit integer to actual speed 12.566/32767=3.834955900753807e-4=0.00038349559007538
  if(omega_right_target>6.283) omega_right_target=6.283;
  else if(omega_right_target<-6.283) omega_right_target=-6.283;
  right_target_send = int(omega_right_target / 0.00019174779503769);   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

  char sT_R = '{';
  byte sH_R = highByte(right_target_send);
  byte sL_R = lowByte(right_target_send);
  char sP_R = '}';
  Serial1.write(sT_R); Serial1.write(sH_R); Serial1.write(sL_R); Serial1.write(sP_R);
}
