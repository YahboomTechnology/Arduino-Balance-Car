 /****************************************************************************
   Yahboom Intelligent Technology Co., Ltd.
   Product Name: Arduino Smart Balance Car
  Product model: BST-ABC ver2.0
  Change record: 181207 increase voltage acquisition display  (liusen)
****************************************************************************/


#include "./PinChangeInt.h"
#include "./MsTimer2.h"
//Speed PID is controlled by speed dial counting
#include "./BalanceCar.h"
#include "./KalmanFilter.h"
#include "./I2Cdev.h"
#include "./MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu; //实例化一个 MPU6050 对象，对象名称为 mpu
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;
//TB6612FNG drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define PinA_left 2  //Interrupt 0
#define PinA_right 4 //Interrupt 1

//Declare custom variables
float Voltage = 0;
int voltagepin = A0;
int time;
byte inByte; //Serial port receive byte
int num;
double Setpoint;                                                  //Angle DIP set point, input, output
double Setpoints, Outputs = 0;                                    //Speed DIP set point, input, output
double kp = 38, ki = 0.0, kd = 0.58;                              //The parameters you need to modify
double kp_speed =3.8, ki_speed = 0.11, kd_speed = 0.0;            //The parameters you need to modify
double kp_turn = 28, ki_turn = 0, kd_turn = 0.29;                 //Rotate PID settings
const double PID_Original[6] = {38, 0.0, 0.58,4.0, 0.12, 0.0};    //Restore default PID parameters
//Rotate PID parameters
double setp0 = 0, dpwm = 0, dl = 0; //Angle balance point, PWM difference, dead zone，PWM1，PWM2
float value;


//********************angle data*********************//
float Q;
float Angle_ax;      //Tilt angle calculated from acceleration
float Angle_ay;
float K1 = 0.05;     //Weighting the value of the accelerometer
float angle0 = 0.00; //Mechanical balance angle
int slong;
//********************angle data*********************//

//***************Kalman_Filter*********************//
float Q_angle = 0.001, Q_gyro = 0.005; //Angle data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; 
float dt = timeChange * 0.001; //PS：The value of dt is the filter sampling time.
//***************Kalman_Filter*********************//

//*********************************************
//******************** speed count ************
//*********************************************

volatile long count_right = 0;
//The volatile long type is used to ensure that the value is valid when the external interrupt pulse count value is used in other functions.
volatile long count_left = 0;
//The volatile long type is used to ensure that the value is valid when the external interrupt pulse count value is used in other functions.
int speedcc = 0;

//////////////////////Pulse calculation/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
/////////////////////////////////////////////////

/////////////////////////////////////////////
int turncount = 0; //Steering intervention time calculation
float turnoutput = 0;
/////////////////////////////////////////////

/////////////////////////////////
#define run_car     '1'
#define back_car    '2'
#define left_car    '3'
#define right_car   '4'
#define stop_car    '0'
/*Car running status enumeration*/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;
int incomingByte = 0;             //Received data byte
String inputString = "";          //Used to store received content
boolean newLineReceived = false;  //Previous data end mark
boolean startBit  = false;        //Agreement start sign
int g_carstate = enSTOP;          //1front2back3left4right0stop
String returntemp = "";            //Store return value
boolean g_autoup = false;
int g_uptimes = 5000;

int front = 0;//front variable
int back = 0;//back variable
int turnl = 0;//Turn left sign
int turnr = 0;//Turn right sign
int spinl = 0;//Left rotation sign
int spinr = 0;//Right rotation sign
int bluetoothvalue;//Bluetooth control
/////////////////////////////////

////////////////////////////////////

int chaoshengbo = 0;
int tingzhi = 0;
int jishi = 0;

////////////////////////////////////


//////////////////////Pulse calculation///////////////////////
void countpluse()
{

  lz = count_left;
  rz = count_right;

  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                //Judgment of the direction of movement of the car.          
                                                                     //Back,PWM is the motor voltage is negative
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))       //Judgment of the direction of movement of the car.          
                                                                 //Advane,PWM is the motor voltage is positive
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                 
                                                              //Judgment of the direction of movement of the car.
                                                              //Right rotation. (The number of right pulses is positive. The number of left pulses is negative.)
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))    //Judgment of the direction of movement of the car.
                                                              //Left rotation. (The number of right pulses is negative. The number of left pulses is positive.)
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  //Determine if the car is lifted
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  //The number of pulses is superimposed, every 5ms into the interrupt
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam = balancecar.pulseright + balancecar.pulseleft;
}
///////////////////////////////////////////



//////////////////////////////////////
void angleout()
{
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD 角度环控制
}
//////////////////////////////////////

//////////////////////////////////////////////////////////
//////////////////////////////////////
/////////////////////////////////////////////////////////
void inter()
{
  sei();
  countpluse();                                     //Pulse superposition subfunction
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC acquires six-axis data of MPU6050 ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);                                   
  angleout();                                       // Angle loop control

  speedcc++;
  if (speedcc >= 8)                                //40ms into the speed loop control
  {
    Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4)                                //40ms into the rotation control
  {
    turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);                              
    turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);                            //小车总PWM输出
 
}
//////////////////////////////////////////////////////////
/////////////////////////////////////
/////////////////////////////////////////////////////////
void SendAutoUp()
{
  g_uptimes --;
  if ((g_autoup == true) && (g_uptimes == 0))
  {

    String CSB;//, VT;
    char temp[10]={0};
    float fgx;
    float fay;
    float leftspeed;
    float rightspeed;
   
    fgx = gx;  
    fay = ay;  
    leftspeed = balancecar.pwm1;
    rightspeed = balancecar.pwm2;
    
    double Gx = (double)((fgx - 128.1f) / 131.0f);              //Angle conversion
    double Ay = ((double)fay / 16384.0f) * 9.8f;
    
   if(leftspeed > 255 || leftspeed < -255)
      return;
   if(rightspeed > 255 || rightspeed < -255)
      return;
   if((Ay < -20) || (Ay > 20))
      return;
   if((Gx < -3000) || (Gx > 3000))
      return; 
      
    returntemp = "";

    memset(temp, 0x00, sizeof(temp));
    dtostrf(leftspeed, 3, 1, temp);  // = %3.2f
    String LV = temp;
    
    memset(temp, 0x00, sizeof(temp));
    dtostrf(rightspeed, 3, 1, temp);  // = %3.1f
    String RV = temp;

    memset(temp, 0x00, sizeof(temp));
    dtostrf(Ay, 2, 2, temp);  // = %2.2f
    String AC = temp;
     
    memset(temp, 0x00, sizeof(temp));
    dtostrf(Gx, 4, 2, temp);  // = %4.2f
    String GY = temp;

     memset(temp, 0x00, sizeof(temp));
    dtostrf(Voltage, 2, 1, temp);  // = %2.1f
    
    String VT = temp;
    
    CSB = "0.00";
    returntemp = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
    Serial.print(returntemp); //Return protocol packet
  }
  
  if (g_uptimes == 0)
      g_uptimes = 5000;
}


// === ===
void setup() {

 // Serial.begin(9600);   
  pinMode(IN1M, OUTPUT);                         //Control the direction of the motor 1, 01 is forward rotation, 10 is reverse rotation
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);                        //Control the direction of the motor 2, 01 is forward rotation, 10 is reverse rotation
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);                        //Left motor PWM
  pinMode(PWMB, OUTPUT);                        //Right motor PWM
  pinMode(STBY, OUTPUT);                        //TB6612FNG enable


  //Initialize motor dirve module
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);  
  pinMode(PinA_right, INPUT);

  // 加入I2C总线
  Wire.begin();                            //Join the I2C bus sequence
  Serial.begin(9600);                     //Open the serial port and set the baud rate to 115200.
  delay(1500);
  mpu.initialize();                       //Initialize MPU6050
  delay(2);
  balancecar.pwm1 = 0;
  balancecar.pwm2 = 0;
  
  //5ms timer interrupt setting, Use Timer2 
  //Note: Using timer2 will affect the PWM output of pin3 pin11, because the PWM uses the timer to control the duty cycle, 
  //so pay attention to check the pin port of the corresponding timer when using the timer.
  MsTimer2::set(5, inter);
  MsTimer2::start();

}

////////////////////////////////////////turn//////////////////////////////////

void ResetPID()
{
  kp = PID_Original[0];
  ki =  PID_Original[1];
  kd =  PID_Original[2];         //you need to modify the parameters
  kp_speed =  PID_Original[3];
  ki_speed =  PID_Original[4];
  kd_speed =  PID_Original[5];  //you need to modify the parameters
}
void ResetCarState()
{
  turnl = 0; 
  turnr = 0;  
  front = 0; 
  back = 0; 
  spinl = 0; 
  spinr = 0; 
  turnoutput = 0;
}

float voltage_test()
{
  float fVoltage;
  int iVoltage;
  pinMode(voltagepin, INPUT);
  iVoltage = analogRead(A0);                        //Get value of A0
  fVoltage = (iVoltage / 1023.0) * 5*4.2-0.35;      //Converted to voltage value
  //Voltage is the ad value (0-1023) collected by port A0.
  //0.35 is the adjustment value due to the accuracy of the resistor.
  return fVoltage;
}
// === ===
void loop() {
  Voltage=voltage_test();
  //Serial.print(Voltage,DEC);
 // Serial.print("\r\n");
  
 String returnstr = "$0,0,0,0,0,0,0,0,0,0,0,0cm,0.0V#";  //Default send

   
    attachInterrupt(0, Code_left, CHANGE);
    attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
   
    //kongzhi(); //Bluetooth interface
    //
  
    //Serial.println(kalmanfilter.angle);
    //Serial.print("\t");
    //Serial.print(bluetoothvalue);
    //Serial.print("\t");
    //      Serial.print( balancecar.angleoutput);
    //      Serial.print("\t");
    //      Serial.print(balancecar.pwm1);
    //      Serial.print("\t");
    //      Serial.println(balancecar.pwm2);
    //      Serial.print("\t");
    //      Serial.println(balancecar.stopr);
    //      Serial.print("\t");
    
    if (newLineReceived)
    {
      switch (inputString[1])
      {
        case run_car:   g_carstate = enRUN;   break;
        case back_car:  g_carstate = enBACK;  break;
        case left_car:  g_carstate = enLEFT;  break;
        case right_car: g_carstate = enRIGHT; break;
        case stop_car:  g_carstate = enSTOP;  break;
        default: g_carstate = enSTOP; break;
      }
      //Determine if the protocol has lost packets
    /* if(inputString.length() < 21)
      {
          
        inputString = "";   // clear the string
        newLineReceived = false;
        //Serial.print(returnstr);
        goto a;
      }*/
      if (inputString[3] == '1' && inputString.length() == 21) //Left shake
      {
        g_carstate = enTLEFT;
        //Serial.print(returnstr);
      }
      else if (inputString[3] == '2' && inputString.length() == 21) //Right shake
      {
        g_carstate = enTRIGHT;
       // Serial.print(returnstr);
      }
  
      if (inputString[5] == '1') //Inquire PID
      {
        char charkp[7], charkd[7], charkpspeed[7], charkispeed[7];
  
        dtostrf(kp, 3, 2, charkp);  // = %3.2f
        dtostrf(kd, 3, 2, charkd);  // = %3.2f
        dtostrf(kp_speed, 3, 2, charkpspeed);  // = %3.2f
        dtostrf(ki_speed, 3, 2, charkispeed);  // = %3.2f
  
        String strkp = charkp; String strkd = charkd; String strkpspeed = charkpspeed; String strkispeed = charkispeed;
  
        returntemp = "$0,0,0,0,0,0,AP" + strkp + ",AD" + strkd + ",VP" + strkpspeed + ",VI" + strkispeed + "#";
  
        Serial.print(returntemp); //Return protocol packet
      }
      else if (inputString[5] == '2') //Restore PID
      {
        ResetPID();
        Serial.print("$OK#"); //Return protocol packet
      }
  
      if (inputString[7] == '1') //automatic reporting
      {
        g_autoup = true;
        Serial.print("$OK#"); //Return protocol packet
      }
      else if (inputString[7] == '2') //stop automatic reporting
      {
        g_autoup = false;
        Serial.print("$OK#"); //Return protocol packet
      }
  
      if (inputString[9] == '1') //Angle loop update $0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26#
      {
        int i = inputString.indexOf("AP");
        int ii = inputString.indexOf(",", i);
        if(ii > i)
        {
          String m_skp = inputString.substring(i + 2, ii);
          m_skp.replace(".", "");
          int m_kp = m_skp.toInt();
          kp = (double)( (double)m_kp / 100.0f);
        }
       
  
        i = inputString.indexOf("AD");
        ii = inputString.indexOf(",", i);
        if(ii > i)
        {
          //ki = inputString.substring(i+2, ii);
          String m_skd = inputString.substring(i + 2, ii);
          m_skd.replace(".", "");
          int m_kd = m_skd.toInt();
          kd = (double)( (double)m_kd / 100.0f);
        }
        Serial.print("$OK#"); //Return protocol packet
      }
  
      if (inputString[11] == '1') //Speed loop update
      {
        int i = inputString.indexOf("VP");
        int ii = inputString.indexOf(",", i);
        if(ii > i)
        {
          String m_svp = inputString.substring(i + 2, ii);
          m_svp.replace(".", "");
          int m_vp = m_svp.toInt();
          kp_speed = (double)( (double)m_vp / 100.0f);
        }
       
  
        i = inputString.indexOf("VI");
        ii = inputString.indexOf("#", i);
        if(ii > i)
        {
          String m_svi = inputString.substring(i + 2, ii);
          m_svi.replace(".", "");
          int m_vi = m_svi.toInt();
          ki_speed = (double)( (double)m_vi / 100.0f);
          Serial.print("$OK#"); //Return protocol packet
        }
       
      }
      //Restore default state
      inputString = "";   // clear the string
      newLineReceived = false;
  
    }
  
a:    switch (g_carstate)
    {
      case enSTOP: turnl = 0; turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
      case enRUN: ResetCarState();front = 250; break;
      case enLEFT: turnl = 1; break;
      case enRIGHT: turnr = 1; break;
      case enBACK: ResetCarState();back = -250; break;
      case enTLEFT: spinl = 1; break;
      case enTRIGHT: spinr = 1; break;
      default: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
    }
  
   //Increase automatic reporting
  SendAutoUp();


}

////////////////////////////////////////pwm///////////////////////////////////



//////////////////////////Pulse interrupt calculation/////////////////////////////////////

void Code_left() {

  count_left ++;

} //Left speed dial count



void Code_right() {

  count_right ++;

} //Right speed dial count

//////////////////////////Pulse interrupt calculation/////////////////////////////////////

//serialEvent() is a new feature in IDE 1.0 and later.
int num1 = 0;
void serialEvent()
{
  
  while (Serial.available())
  {
    incomingByte = Serial.read();              //Read one byte by byte, put the read data into a string array to form a completed packet
    if (incomingByte == '$')
    {
      num1 = 0;
      startBit = true;
    }
    if (startBit == true)
    {
      num1++;
      inputString += (char) incomingByte;     //Full-duplex serial port does not need to add delay below, half-duplex  need to add delay below
    }
    if (startBit == true && incomingByte == '#')
    {
      newLineReceived = true;
      startBit = false;
    }
    
    if(num1 >= 80)
    {
      num1 = 0;
      startBit = false;
      newLineReceived = false;
      inputString = "";
    }	
  }
}


