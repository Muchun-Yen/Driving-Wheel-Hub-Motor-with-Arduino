/************************************************************
[Notation]
* The program is for RIGHT wheel hub motor control
* The Vq/Vd max/min value is 700~ -700
* MESSAGE_SHOW frequency is depence received HW device, in my case I use a dongle which can not received high frequency messages and send a command at same time. Please don't reference this value, because my device is a very slow device.
/************************************************************
[Control Board Communication Format]
Send Command Format: 7 Byte
    Bytes:  Start  Vq_Hbyte Vq_Lbyte Vd_Hbyte Vd_Lbyte Checkcode  End
ex:(Vq=300, Vd=4)
            0x7B    0x01    0x2C      0x00    0x04      0x7C      0x7D
ex:(Vq=0, Vd=0)
            0x7B    0x00    0x00      0x00    0x00      0x55      0x7D

Receive Data Format: 9 Byte
           [Byte:1] [Byte:2]  [Byte:3]  [Byte:4]  [Byte:5]  [Byte:6]    [Byte:7]    [Byte:8]  [Byte:9]
    Bytes:  Start    Vq_Hbyte  Vq_Lbyte  Vd_Hbyte  Vd_Lbyte  pulse/ms_H  pulse/ms_L  Checkcode  End
    ex:    0x7B                                                                                 0x7D
/*************************************************************
[ Encode value check ]
 *      Bit         LSB           MSB
 *                  A     B       C
 *      Pin         2     3       21 (Arduino Mega 2560 Board)
 *      WireColor   Blue  Green   Yellow
 *      ClockWise value:=1,3,2,6,4,5
 *      CountClockWise:=1,5,4,6,2,3
/*************************************************************
[HW Arduino Mega 2560]
Serial port (Default serial for debug )
Serial1 port (connect to Motor control board Right wheel)
Serial3 port (connect to the upper drive Serial1 port)
*************************************************************/

#define VQ_MAX 700
#define VQ_MIN -700
#define MESSAGE_SHOW 1000//ms

union Data_Setting {
  struct _ByteSet {
    byte L;
    byte H;
  } Byte;
  int Data;
};

union Data_Setting MotorDataR[3];
byte getDataR[8];
byte sendDataR[7] = {123, 0, 0, 0, 0, 0, 125};
byte sendDataStop[7] = {123, 0, 0, 0, 0, 85, 125};

unsigned long PastTime = 0;
int vq = 0, vd = 0, checksum = 0;

int encodePinA = 2;
int encodePinB = 3;
int encodePinC = 21;
volatile long Encoderpos = 0;
long EncoderposPre = 0;
volatile int lastEncoded = 0;

#define LOOPTIME        100
unsigned long lastMilli = 0;
long dT = 0;


int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;
int pinCState = 0;
int pinCStateOld = 0;


double omega_target = 0;
double omega_actual = 0;
int CPR = 60;                                                                     //encoder count per revolution
int gear_ratio = 1;
int actual_send = 0;
int target_receive = 0;

float Kp = 0.9;
float Ki = 0.005;
double error;
double pidTerm = 0;
double sum_error = 0;

double calculated_pidTerm;
double constrained_pidterm;

//test for encoder error
int EncodeDiff = 0;
int EncodeDiffPre = 0;


//restart condition for Warrning Encoder problem
int StartFlag = 0;
int receiveVq = 0;
byte receiveMode = 0;
byte actualMode = 0;
int vq_Feedback = 0;

void readCmd_wheel_angularVel(){
  if (Serial3.available()){
    char rT = (char)Serial3.read();                                               //read target speed from mega

    if (rT == 'm'){                                                               //test by BT
      StartFlag = 1;
      Encoderpos = 0;
      EncoderposPre = 0;
      omega_target = 0;
      vq = 0;
    }

    else if (rT == '{'){
      byte commandArray[3];
      Serial3.readBytes(commandArray, 3);
      byte rH = commandArray[0];
      byte rL = commandArray[1];
      char rP = commandArray[2];

      if (rP == '}'){
        target_receive = (rH << 8) + rL;
        omega_target = double (target_receive * 0.00006103701895199438);          //convert received 16 bit integer to actual speed
      }
    }

    else if (rT == '['){
      byte commandArray_x[4];
      Serial3.readBytes(commandArray_x, 4);
      byte rH_x = commandArray_x[0];
      byte rL_x = commandArray_x[1];
      byte rP_x = commandArray_x[2];
      char rQ_x = commandArray_x[3];
      
      if (rQ_x == ']'){
        receiveVq = int((rH_x << 8) + rL_x);
        receiveMode = rP_x;

        if (receiveMode == 0xAF)
          StartFlag = 2;
        else if (receiveMode == 0x05)
          StartFlag = 1;
        else if (receiveMode == 0x77){
          StartFlag = 3;
          omega_target = 0.5;
        }
      }
    }
  }    //end of if (Serial3.available())
}

void sendFeedback_wheel_voltageQ(){
  char sT_x = '[';                                                                //send start byte
  byte sH_x = highByte(vq_Feedback);                                              //send high byte
  byte sL_x = lowByte(vq_Feedback);                                               //send low byte
  byte sQ_x = actualMode;                                                         //send mode byte
  char sP_x = ']';                                                                //send stop byte
  Serial3.write(sT_x); Serial3.write(sH_x); Serial3.write(sL_x); Serial3.write(sQ_x); Serial3.write(sP_x);
}


void sendFeedback_wheel_angularVel(){
  actual_send = int(omega_actual / 0.00006103701895199438);                       //convert rad/s to 16 bit integer to send 6.103701895199438e-5
  char sT = '{';                                                                  //send start byte
  byte sH = highByte(actual_send);                                                //send high byte
  byte sL = lowByte(actual_send);                                                 //send low byte
  char sP = '}';                                                                  //send stop byte
  Serial3.write(sT); Serial3.write(sH); Serial3.write(sL); Serial3.write(sP);
}

void getMotorData(){
/* omega_actual = ((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  */
  omega_actual = (double)((Encoderpos - EncoderposPre) * (1000 / dT)) / CPR;      //ticks/s to rad/s
  EncodeDiff = abs(Encoderpos - EncoderposPre);
  EncoderposPre = Encoderpos;
}

double updatePid(double targetValue, double currentValue){
  static double last_error = 0;
  error = targetValue - currentValue;

  Kp = 0.15;
  Ki = 0.005;
  sum_error = sum_error + error * dT;
  sum_error = constrain(sum_error, -400, 400);

  pidTerm = Kp * error + Ki * sum_error;

  calculated_pidTerm = pidTerm / 0.0028571428571429;                              // 2/700=0.0028571428571429
  constrained_pidterm = constrain(calculated_pidTerm, -700, 700);
  return constrained_pidterm;
}


void sendCmd(){
  if (vq >= VQ_MAX)
    vq = VQ_MAX;
  else if (vq <= VQ_MIN)
    vq = VQ_MIN;

  sendDataR[1] = highByte(vq);
  sendDataR[2] = lowByte(vq);
  sendDataR[3] = highByte(vd);
  sendDataR[4] = lowByte(vd);
  sendDataR[5] = (0x55 ^ sendDataR[1] ^ sendDataR[2] ^ sendDataR[3] ^ sendDataR[4]);

  Serial1.write(sendDataR, 7);
}


void doEncoder(){
  pinAState = digitalRead(encodePinA);
  pinBState = digitalRead(encodePinB);
  pinCState = digitalRead(encodePinC);

  if (pinAState == 1 && pinBState == 0 && pinCState == 0){                        //value:=1
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCState == 1)                 //value:=5 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1 && pinCState == 0)                 //value:=3 // CCW
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 1 && pinCState == 0){                        //value:=3
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCState == 0)                 //value:=1 // CW
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCState == 0)                 //value:=2 // CCW
      Encoderpos --;
  }

  if (pinAState == 0 && pinBState == 1 && pinCState == 0){                        //value:=2
    if (pinAStateOld == 1 && pinBStateOld == 1 && pinCState == 0)                 //value:=3 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCState == 1)                 //value:=6 // CCW
      Encoderpos --;
  }

  if (pinAState == 0 && pinBState == 1 && pinCState == 1){                        //value:=6
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCState == 0)                 //value:=2 // CW
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0 && pinCState == 1)                 //value:=4 // CCW
      Encoderpos --;
  }

  if (pinAState == 0 && pinBState == 0 && pinCState == 1){                        //value:=4
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCState == 1)                 //value:=6 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCState == 1)                 //value:=5 // CCW
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 0 && pinCState == 1){                        //value:=5
    if (pinAStateOld == 0 && pinBStateOld == 0 && pinCState == 1)                 //value:=4 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCState == 0)                 //value:=1 // CCW
      Encoderpos --;
  }

  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
  pinCStateOld = pinCState;
}

void setup(){
  Serial3.begin(115200);                                                          //for upper device sned/receive command
  Serial1.begin(115200);                                                          //for commect to Right wheel control board
  Serial.begin(115200);                                                           //default serial port for debug

  pinMode(encodePinA, INPUT);
  pinMode(encodePinB, INPUT);
  pinMode(encodePinC, INPUT);
  digitalWrite(encodePinA, HIGH);                                                 //turn on pullup resistor
  digitalWrite(encodePinB, HIGH);                                                 //turn on pullup resistor
  digitalWrite(encodePinC, HIGH);                                                 //turn on pullup resistor

  attachInterrupt(0, doEncoder, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
  attachInterrupt(2, doEncoder, CHANGE);                                          //encoder pin on interrupt 2 - pin 21
}


void loop(){
  readCmd_wheel_angularVel();                                                     //include the initial command
  if ((millis() - lastMilli) >= LOOPTIME){
    dT = millis() - lastMilli;
    lastMilli = millis();

    if (StartFlag == 1){
      getMotorData();
      sendFeedback_wheel_angularVel();                                            //send actually speed to mega
      vq = (updatePid(omega_target, omega_actual));                               //compute vq value from rad/s

      actualMode = 0x05;
      vq_Feedback = vq;
      sendFeedback_wheel_voltageQ();                                              //send vq and mode

      if (omega_target == 0){
        vq = 0;
        sum_error = 0;
      }

      if (vq == 0)
        Serial.println("Warrning!!!Vq is 0");

      sendCmd();
    }

    else if (StartFlag == 2){
      getMotorData();                                                             // calculate speed
      sendFeedback_wheel_angularVel();                                            //send actually speed to mega

      vq_Feedback = receiveVq;
      vq = vq_Feedback;

      actualMode = 0xAF;
      sendFeedback_wheel_voltageQ();                                              //send vq and mode

      sum_error = 0;

      if (vq == 0)
        Serial.println("Warrning!!!Vq is 0");
      sendCmd();
    }

/* test only */
    else if (StartFlag == 3){
      getMotorData();
      sendFeedback_wheel_voltageQ();                                              //send vq and mode

      vq_Feedback = 127;
      vq = vq_Feedback;

      actualMode = 0x77;
      sendFeedback_wheel_voltageQ();                                              //send vq and mode

      sum_error = 0;

      if (vq == 0)
        Serial.println("Warrning!!!Vq is 0");
      sendCmd();
    }
  }
}