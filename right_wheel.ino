/************************************************************
[Version]
20160701
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

//Kalman
#include <math.h>

class KalmanFilter
{
  public:

    KalmanFilter(double q, double r);
    double Update(double);
    double GetK() {
      return k;
    }

  private:

    double k; //kalman gain
    double p; //estimation error covariance
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
};

KalmanFilter::KalmanFilter(double q, double r): q(q), r(r), x(0.0)
{
  p = sqrt(q * q + r * r);
}

double KalmanFilter::Update(double value)
{
  p += q;
  k = p / (p + r);
  x += k * (value - x);
  p *= (1 - k);

  return x;
}
KalmanFilter kalman(0.1, 1.0);

//limit 1 rev/sec
#define VQ_MAX 260
#define VQ_MIN -260

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

#define LOOPTIME        100 //500 for test & draw
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
int CPR = 90;                                                                     //encoder count per revolution
int gear_ratio = 1;
int actual_send = 0;
int target_receive = 0;

float Kp = 0.9;
float Ki = 0.005;
double error;
double pidTerm = 0;
double sum_error = 0;
double Kalman = 0;

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

//Add Vd control
int VD_ENABLE_LIMITE = 160;
int VD_ENABLE_VALUE = 50;

void readCmd_wheel_angularVel() {
  if (Serial3.available()) {
    char rT = (char)Serial3.read();                                               //read target speed from mega

    if (rT == 'm') {                                                              //test by BT
      StartFlag = 1;
      Encoderpos = 0;
      EncoderposPre = 0;
      omega_target = 0;
      sum_error = 0;
      vq = 0;
      vd = 0xFFFF;
      Serial.println("BLDC Enable");
      sendDataR[1] = highByte(vq);
      sendDataR[2] = lowByte(vq);
      sendDataR[3] = highByte(vd);
      sendDataR[4] = lowByte(vd);
      sendDataR[5] = (0x55 ^ sendDataR[1] ^ sendDataR[2] ^ sendDataR[3] ^ sendDataR[4]);
      Serial1.write(sendDataR, 7);
      vd = 0x0;
    }

    else if (rT == 'k') {                                                              //test by BT
      StartFlag = 1;
      Encoderpos = 0;
      EncoderposPre = 0;
      omega_target = 0;
      sum_error = 0;
      vq = 0;
      vd = 0xAAAA;
      Serial.println("BLDC Disable");
      sendDataR[1] = highByte(vq);
      sendDataR[2] = lowByte(vq);
      sendDataR[3] = highByte(vd);
      sendDataR[4] = lowByte(vd);
      sendDataR[5] = (0x55 ^ sendDataR[1] ^ sendDataR[2] ^ sendDataR[3] ^ sendDataR[4]);
      Serial1.write(sendDataR, 7);
      vd = 0x0;
    }

    else if (rT == '{') {
      byte commandArray[3];
      Serial3.readBytes(commandArray, 3);
      byte rH = commandArray[0];
      byte rL = commandArray[1];
      char rP = commandArray[2];

      if (rP == '}') {
        target_receive = (rH << 8) + rL;

        //limit 1 rev/sec
        omega_target = double (target_receive * 0.0001917477950376904);          //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
      }
    }

  }    //end of if (Serial3.available())
}


void sendFeedback_wheel_angularVel() {

  //limit 1 rev/sec
  actual_send = int(omega_actual / 0.0001917477950376904);          //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
  char sT = '{';                                                                  //send start byte
  byte sH = highByte(actual_send);                                                //send high byte
  byte sL = lowByte(actual_send);                                                 //send low byte
  char sP = '}';                                                                  //send stop byte
  Serial3.write(sT); Serial3.write(sH); Serial3.write(sL); Serial3.write(sP);
}


void getMotorData() {
  EncodeDiff = Encoderpos - EncoderposPre;

  //for 100ms limitatiopn in 6.28 rads/sec
  if (EncodeDiff >= 9)
    EncodeDiff = 9;
  if (EncodeDiff <= -9)
    EncodeDiff = -9;

  //Kalman
  Kalman = ((EncodeDiff) * (1000 / dT)) * 2 * PI / (CPR);
  omega_actual = kalman.Update(Kalman);

  //  Encoderpos=0;//recount
  EncoderposPre = Encoderpos;
  Serial.println(String("EncodeDiff=") + " " + String(EncodeDiff));
  Serial.println(String("dT=") + " " + String(dT));
}
/*
//open loop
double updatePid(double targetValue, double currentValue) {

//limit 1 rev/sec
  calculated_pidTerm = targetValue / 0.024165;                              // 6.283 / 260 =0.0241653846153846
  constrained_pidterm = constrain(calculated_pidTerm, -260, 260);

  Serial.println(String("constrained_pidterm=") + " " + String(constrained_pidterm));

  return constrained_pidterm;
}
*/

//close loop
double updatePid(double targetValue, double currentValue) {
  static double last_error = 0;
  error = targetValue - currentValue;

  Kp = 0.15;
  Ki = 0.005;
  sum_error = sum_error + error * dT;
  sum_error = constrain(sum_error, -2000, 2000);
  //  Serial.println(String("sum_error is =")+" "+String(sum_error));
  pidTerm = Kp * error + Ki * sum_error;

  //limit 1 rev/sec
  calculated_pidTerm = pidTerm / 0.024165;                             // 6.283 / 260 =0.0241653846153846
  constrained_pidterm = constrain(calculated_pidTerm, -260, 260);
  return constrained_pidterm;
}


void sendCmd() {
  if (vq >= VQ_MAX)
    vq = VQ_MAX;
  else if (vq <= VQ_MIN)
    vq = VQ_MIN;

  if ((abs(vq) <= VD_ENABLE_LIMITE) && (vq != 0))
  {
    vd = VD_ENABLE_VALUE;                                                         // The Vd always be postive value, even vq is a negative value
    digitalWrite(51, HIGH);                                                       // turn the LED on (HIGH is the voltage level)
  }
  else
  {
    vd = 0;
    digitalWrite(51, LOW);                                                        // turn the LED off by making the voltage LOW
  }
  sendDataR[1] = highByte(vq);
  sendDataR[2] = lowByte(vq);
  sendDataR[3] = highByte(vd);
  sendDataR[4] = lowByte(vd);
  sendDataR[5] = (0x55 ^ sendDataR[1] ^ sendDataR[2] ^ sendDataR[3] ^ sendDataR[4]);

  Serial1.write(sendDataR, 7);
}


void revFromMCU() {
  if (Serial1.available()) {
    if (Serial1.read() == '{') {
      Serial1.readBytes(getDataR, 8);
      if (getDataR[7] == '}') {
        checksum = (0x55 ^ getDataR[0] ^ getDataR[1] ^ getDataR[2] ^ getDataR[3] ^ getDataR[4] ^ getDataR[5]);
        if (checksum == getDataR[6]) {
          for (int i = 0; i < 3; i ++) {
            MotorDataR[i].Byte.L = getDataR[2 * i];
            MotorDataR[i].Byte.H = getDataR[2 * i + 1];
          }
          Serial.println(String("MCU Vq=") + " " + \
                         String(MotorDataR[0].Data) + " " + \
                         String("MCU Vd=") + " " + \
                         String(MotorDataR[1].Data) + " " + \
                         String("MCU rds=") + " " + \
                         String(MotorDataR[2].Data));
        }
      }
    }
  }
}


void doEncoder() {
  pinAState = digitalRead(encodePinA);
  pinBState = digitalRead(encodePinB);
  pinCState = digitalRead(encodePinC);

  if (pinAState == 0 && pinBState == 1 && pinCState == 1) {                       //value:=1
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=5 // CW
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=3 // CCW
      Encoderpos --;
  }

  if (pinAState == 0 && pinBState == 0 && pinCState == 1) {                       //value:=3
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 1)              //value:=1 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=2 // CCW
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 0 && pinCState == 1) {                       //value:=2
    if (pinAStateOld == 0 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=3 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 0)              //value:=6 // CCW
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 0 && pinCState == 0) {                       //value:=6
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=2 // CW
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=4 // CCW
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 1 && pinCState == 0) {                       //value:=4
    if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 0)              //value:=6 // CW
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=5 // CCW
      Encoderpos --;
  }

  if (pinAState == 0 && pinBState == 1 && pinCState == 0) {                       //value:=5
    if (pinAStateOld == 1 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=4 // CW
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 1)              //value:=1 // CCW
      Encoderpos --;
  }

  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
  pinCStateOld = pinCState;
}

void setup() {
  Serial3.begin(115200);                                                          //for upper device sned/receive command
  Serial1.begin(115200);                                                          //for commect to Right wheel control board
  Serial.begin(115200);                                                           //default serial port for debug

  pinMode(51, OUTPUT);                                                           //for VD_ENABLE_VALUE check

  pinMode(encodePinA, INPUT);
  pinMode(encodePinB, INPUT);
  pinMode(encodePinC, INPUT);
  digitalWrite(encodePinA, HIGH);                                                 //turn on pullup resistor
  digitalWrite(encodePinB, HIGH);                                                 //turn on pullup resistor
  digitalWrite(encodePinC, HIGH);                                                 //turn on pullup resistor

  pinAStateOld = digitalRead(encodePinA);
  pinBStateOld = digitalRead(encodePinB);
  pinCStateOld = digitalRead(encodePinC);

  attachInterrupt(0, doEncoder, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
  attachInterrupt(2, doEncoder, CHANGE);                                          //encoder pin on interrupt 2 - pin 21
}


void loop() {
  readCmd_wheel_angularVel();                                                     //include the initial command

  if ((millis() - lastMilli) >= LOOPTIME) {
    dT = millis() - lastMilli;
    lastMilli = millis();

    getMotorData();
    sendFeedback_wheel_angularVel();                                            //send actually speed to mega
    vq = (updatePid(omega_target, omega_actual));                               //compute vq value from rad/s

    if (omega_target == 0) {
      vq = 0;
      sum_error = 0;
    }

    if (vq == 0)
      Serial.println("Warrning!!!Vq is 0");

    sendCmd();
  }
}