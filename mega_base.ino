/*************************************************************
[HW Arduino Mega 2560]
Serial port (Default serial for Connect ROSSerial )
Serial1 port (connect to Motor control board Right wheel)
Serial2 port (connect to Motor control board Left wheel)
Serial3 port (connect to BT (Test only))
*************************************************************/

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
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

char commandArray_L_x[4];
byte sT_L_x = 0;  //send start byte
byte sH_L_x = 0;  //send high byte
byte sL_L_x = 0;  //send low byte
byte sP_L_x = 0;  //send stop byte

byte rT_L_x = 0;  //receive start byte
byte rH_L_x = 0;  //receive high byte
byte rL_L_x = 0;  //receive low byte
byte rP_L_x = 0;  //receive stop byte

char commandArray_R_x[4];
byte sT_R_x = 0;  //send start byte
byte sH_R_x = 0;  //send high byte
byte sL_R_x = 0;  //send low byte
byte sP_R_x = 0;  //send stop byte

byte rT_R_x = 0;  //receive start byte
byte rH_R_x = 0;  //receive high byte
byte rL_R_x = 0;  //receive low byte
byte rP_R_x = 0;  //receive stop byte

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

double omega_left_target_vq = 0.0;
double omega_right_target_vq = 0.0;
double omega_left_mode = 0.0;
double omega_right_mode = 0.0;

int left_actual_receive_vq = 0;
int left_target_send_vq = 0;
int right_actual_receive_vq = 0;
int right_target_send_vq = 0;


byte left_mode_target_send = 0;                                                      //0xAF:=Open Loop, 0x05:=Close Loop
int left_mode_actual_receive = 0;                                                    //2:=open loop, 1:=close loop
byte right_mode_target_send = 0;                                                     //0xAF:=Open Loop, 0x05:=Close Loop
int right_mode_actual_receive = 0;                                                   //2:=open loop, 1:=close loop

ros::NodeHandle nh;

geometry_msgs::Vector3 vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);

geometry_msgs::Quaternion vq_msg;
ros::Publisher pvq("feedback_wheel_voltageQ", &vq_msg);

void messageCb_vq(const geometry_msgs::Quaternion& msg_x){
  omega_left_target_vq      = msg_x.x;
  omega_right_target_vq     = msg_x.y;
  omega_left_mode           = msg_x.z;
  omega_right_mode          = msg_x.w;
}

void messageCb(const geometry_msgs::Vector3& msg){
  omega_left_target = msg.x;
  omega_right_target = msg.y;
}

ros::Subscriber<geometry_msgs::Quaternion> svq("cmd_wheel_voltageQ", messageCb_vq);
ros::Subscriber<geometry_msgs::Vector3> s("cmd_wheel_angularVel", messageCb);

void setup(){
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);

  nh.subscribe(svq);
  nh.advertise(pvq);

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

    sendCmd_wheel_voltageQ_L();
    sendCmd_wheel_voltageQ_R();

    vq_msg.x = left_actual_receive_vq;
    vq_msg.y = right_actual_receive_vq;
    vq_msg.z = left_mode_actual_receive;
    vq_msg.w = right_mode_actual_receive;

    pvq.publish(&vq_msg);
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
        omega_left_actual = double (left_actual_receive * 0.00006103701895199438);   //convert received 16 bit integer to actual speed
      }
    }

    else if (rT_L == '['){
      char commandArray_L_x[4];
      Serial2.readBytes(commandArray_L_x, 4);
      byte rH_L_x = commandArray_L_x[0];
      byte rL_L_x = commandArray_L_x[1];
      byte rQ_L_x = commandArray_L_x[2];
      char rP_L_x = commandArray_L_x[3];

      if (rP_L_x == ']'){
        left_actual_receive_vq = (rH_L_x << 8) + rL_L_x;

        if (rQ_L_x == 0xAF)
          left_mode_actual_receive = 2;

        else if (rQ_L_x == 0x05)
          left_mode_actual_receive = 1;

        else if (rQ_L_x == 0x77)
          left_mode_actual_receive = 3;

        else
          left_mode_actual_receive = 0;
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
        omega_right_actual = double (right_actual_receive * 0.00006103701895199438); //convert received 16 bit integer to actual speed
      }
    }

    else if (rT_R == '['){
      char commandArray_R_x[4];
      Serial1.readBytes(commandArray_R_x, 4);
      byte rH_R_x = commandArray_R_x[0];
      byte rL_R_x = commandArray_R_x[1];
      byte rQ_R_x = commandArray_R_x[2];
      char rP_R_x = commandArray_R_x[3];

      if (rP_R_x == ']'){
        right_actual_receive_vq   = (rH_R_x << 8) + rL_R_x;

        if (rQ_R_x == 0xAF)
          right_mode_actual_receive = 2;

        else if (rQ_R_x == 0x05)
          right_mode_actual_receive = 1;

        else if (rQ_R_x == 0x77)
          right_mode_actual_receive = 3;

        else
          right_mode_actual_receive = 0;
      }
    }
  }
}


void sendCmd_wheel_angularVel_L(){
  left_target_send = int(omega_left_target / 0.00006103701895199438);                //convert rad/s to 16 bit integer to send
  char sT_L = '{'; //send start byte
  byte sH_L = highByte(left_target_send);
  byte sL_L = lowByte(left_target_send);
  char sP_L = '}';
  Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_angularVel_R(){
  right_target_send = int(omega_right_target / 0.00006103701895199438);              //convert rad/s to 16 bit integer to send
  char sT_R = '{';
  byte sH_R = highByte(right_target_send);
  byte sL_R = lowByte(right_target_send);
  char sP_R = '}';
  Serial1.write(sT_R); Serial1.write(sH_R); Serial1.write(sL_R); Serial1.write(sP_R);
}

void sendCmd_wheel_voltageQ_L(){
  left_target_send_vq = int(omega_left_target_vq );
 
  if (omega_left_mode == 2)
    left_mode_target_send = 0xAF;

  else if (omega_left_mode == 1)
    left_mode_target_send = 0x05;

  else if (omega_left_mode == 3)
    left_mode_target_send = 0x77;

  else
    left_mode_target_send = 0x0;

  char sT_L_x = '[';
  byte sH_L_x = highByte(left_target_send_vq);
  byte sL_L_x = lowByte(left_target_send_vq);
  byte sQ_L_x = left_mode_target_send;
  char sP_L_x = ']';
  Serial2.write(sT_L_x); Serial2.write(sH_L_x); Serial2.write(sL_L_x); Serial2.write(sQ_L_x); Serial2.write(sP_L_x);
}


void sendCmd_wheel_voltageQ_R(){
  right_target_send_vq = int(omega_right_target_vq);

  if (omega_right_mode == 2)
    right_mode_target_send = 0xAF;

  else if (omega_right_mode == 1)
    right_mode_target_send = 0x05;

  else if (omega_right_mode == 3)
    right_mode_target_send = 0x77;

  else
    right_mode_target_send = 0x0;

  char sT_R_x = '[';
  byte sH_R_x = highByte(right_target_send_vq);
  byte sL_R_x = lowByte(right_target_send_vq);
  byte sQ_R_x =  right_mode_target_send;
  char sP_R_x = ']';
  Serial1.write(sT_R_x); Serial1.write(sH_R_x); Serial1.write(sL_R_x); Serial1.write(sQ_R_x); Serial1.write(sP_R_x);
}
