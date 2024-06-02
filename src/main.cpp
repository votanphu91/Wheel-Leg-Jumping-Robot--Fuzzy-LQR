#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"
#include "encoder.h"
#include "motorControl.h"
#include "MPU6050.h"
#include <EEPROM.h>
#define EEPROM_SIZE 24        // Each float is 4 bytes, 6 floats * 4 bytes = 24 bytes
#define ToRad PI/180
#define ToDeg 180/PI
#define factortheta PI/20
#define factorphi PI/180
#define LED 15
#define LOA 2
float leftvolt; //output volt left motor in LQR
float righvolt; //output volt right motor in lQR

int channel_PWM1 = 3;        // su dung 2 kenh de kiem soat huong va toc do cua dong co , // PWM voi 8 kenh dau voi tan so 80MHz (high-speed channel), và 8 kênh cuối clock 1MHz (low-speed channel).
int channel_PWM2 = 4;       // co 16 kenh ledc co san, ta co the lua chon bat ki tu 0 - 15
int channel_PWM3 = 5;
int channel_PWM4 = 6;
// bien toan cuc
long PWML = 0, PWMR = 0;              // Tinh so xung can cap cho banh trai va phai
float k1, k2, k3, k4, k5, k6;
bool falldown;
float mpudata = 0;
float theta = 0, psi = 0, phi = 0;
float thetadot, psidot, phidot;
float thetaold, psiold, phiold;
float addtheta;
float addphi;
int ForwardBack;
int LeftRight;
// Khoi tao Timer 
uint32_t timer;
int inChar;
String myString;
unsigned long prevTime_T1 ;
// bien tao xung cho DC
int freq_PWM = 5000;    //10khz pwm xac dinh khoang thoi gian 
int resolution_PWM = 8;//2^10 + 1 = 1025    // Dat do phan giai cua PWM 8, 10, 12 BIT 
BluetoothSerial SerialBT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
String pass = "12345678";
String Data;
void Loa_setup(){
  pinMode(LED, OUTPUT);
  pinMode(LOA, OUTPUT);
  digitalWrite(LED, LOW);
  digitalWrite(LOA, LOW);

  // Phát âm thanh khi khởi động
  digitalWrite(LOA, HIGH);
  delay(500);
  digitalWrite(LOA, LOW);
  delay(500);
  digitalWrite(LOA, HIGH);
  delay(500);
  digitalWrite(LOA, LOW);
  delay(500);
  digitalWrite(LOA, HIGH);
  delay(500);
  digitalWrite(LOA, LOW);
}void docgiatri_eeprom(){
  EEPROM.begin(EEPROM_SIZE);
  // Đọc giá trị K từ EEPROM
  k1 = EEPROM.readFloat(0);
  k2 = EEPROM.readFloat(4);
  k3 = EEPROM.readFloat(8);
  k4 = EEPROM.readFloat(12);
  k5 = EEPROM.readFloat(16);
  k6 = EEPROM.readFloat(20);
}
void setup() {  
    Serial.begin(115200);
    SerialBT.begin("BKTECH GROUP");    // Khoi tao kết nối Bluetooth
    Loa_setup();
    docgiatri_eeprom();
    pinMode(ENCODER_1A,INPUT_PULLUP);
    pinMode(ENCODER_1B,INPUT_PULLUP);
    pinMode(ENCODER_2A,INPUT_PULLUP);
    pinMode(ENCODER_2B,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_1A),encoder1_isr,RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2A),encoder2_isr,RISING);
    ledcSetup(channel_PWM1,freq_PWM,resolution_PWM);
    ledcAttachPin(IN1, channel_PWM1);
    ledcSetup(channel_PWM2,freq_PWM,resolution_PWM);
    ledcAttachPin(IN2, channel_PWM2);
    ledcSetup(channel_PWM3,freq_PWM,resolution_PWM);
    ledcAttachPin(IN3, channel_PWM3);
    ledcSetup(channel_PWM4,freq_PWM,resolution_PWM);
    ledcAttachPin(IN4, channel_PWM4);
    setup_MPU6050();
      // Set factor of K maxtrix
    // K = | k1   k2   k3   k4   k5   k6 |
    //     | k1   k2   k3   k4  -k5  -k6 |
    // k1 = 28;                        //k1*theta
    // k2 = 37;                        //k2*thetadot
    // k3 = 1200;                       //k3*psi
    // k4 = 17;                        //k4*psidot
    // k5 = 37;                        //k5*phi  
    // k6 = 2;                         //k6*phidot
    ForwardBack = 0, LeftRight = 0, addphi = 0, addtheta = 0, mpudata = 0, theta = 0, psi = 0, phi = 0;

} 
void readMPU(){
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  mpudata = KalmanAngleRoll;
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
  if(abs(mpudata) > 30){
    falldown = true;
    leftencoder = 0;
    rightencoder = 0;
    //Reset zero setpoint
    addtheta = 0;
    addphi = 0;
  }else{
    falldown = false;
  }
}
//Read theta angle function//
float gettheta(long lencoder, long rencoder) {  //deg value
  float angle = (0.5*360*(lencoder + rencoder))/333;//// 0.5*360/333
  return angle; 
}
//Read phi angle function//
float getphi(long lencoder, long rencoder) {    //deg value
  float angle = (32.5/232)*(lencoder - rencoder);//(R/W) (32.5/232)
  return angle;
}
//LQR function//
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_) {
  leftvolt = k1*theta_ + k2*thetadot_ + k3*psi_ + k4*psidot_ - k5*phi_ - k6*phidot_;
  righvolt = k1*theta_ + k2*thetadot_ + k3*psi_ + k4*psidot_ + k5*phi_ + k6*phidot_;

  PWML = map(leftvolt, -(k3*PI)/15, (k3*PI)/15, -250, 250);//Limit 15 deg.
  PWMR = map(righvolt, -(k3*PI)/15, (k3*PI)/15, -250, 250);

  PWML = constrain(PWML, -240, 240);//limit pwm value in (-240, 240) because we using high frequency pwm (31 khz)
  PWMR = constrain(PWMR, -240, 240);
}
void serialEvent() {
  while (Serial.available() > 0) 
  {
    inChar = Serial.read();
  }
  //Control motor forward
  if(inChar == 70)//F -->run
  {
    ForwardBack = 1;
  }
  if(inChar == 84)//T -->stop
  {
    ForwardBack = 0;
  }
  //Control motor Back
  if(inChar == 66)//B
  {
    ForwardBack = -1;
  }
  if(inChar == 86)//V
  {
    ForwardBack = 0;
  }
  //Control motor Left
  if(inChar == 76)//L
  {
    LeftRight = 1;
  }
  if(inChar == 85)//U
  {
    LeftRight = 0;
  }
  //Control motor right
  if(inChar == 82)//R
  {
    LeftRight = -1;
  }
  if(inChar == 79)//O
  {
    LeftRight = 0;
  }
}

void parseData(String data) {
  int index1 = data.indexOf(','); // Find the first comma
  int index2 = data.indexOf(',', index1 + 1); // Find the second comma
  int index3 = data.indexOf(',', index2 + 1); // Find the third comma
  int index4 = data.indexOf(',', index3 + 1); // Find the fourth comma
  int index5 = data.indexOf(',', index4 + 1); // Find the fifth comma

  if (index1 > 0 && index2 > index1 && index3 > index2 && index4 > index3 && index5 > index4) { // Ensure that all commas are found
    float newK1 = data.substring(0, index1).toFloat();
    float newK2 = data.substring(index1 + 1, index2).toFloat();
    float newK3 = data.substring(index2 + 1, index3).toFloat();
    float newK4 = data.substring(index3 + 1, index4).toFloat();
    float newK5 = data.substring(index4 + 1, index5).toFloat();
    float newK6 = data.substring(index5 + 1).toFloat();

    // Check if any value has changed
    if (newK1 != k1 || newK2 != k2 || newK3 != k3 || newK4 != k4 || newK5 != k5 || newK6 != k6) {
      k1 = newK1;
      k2 = newK2;
      k3 = newK3;
      k4 = newK4;
      k5 = newK5;
      k6 = newK6;

      // Save new values to EEPROM
      EEPROM.writeFloat(0, k1);
      EEPROM.writeFloat(4, k2);
      EEPROM.writeFloat(8, k3);
      EEPROM.writeFloat(12, k4);
      EEPROM.writeFloat(16, k5);
      EEPROM.writeFloat(20, k6);
      EEPROM.commit();

      // Print values to verify
      Serial.print("k1: "); Serial.println(k1);
      Serial.print("k2: "); Serial.println(k2);
      Serial.print("k3: "); Serial.println(k3);
      Serial.print("k4: "); Serial.println(k4);
      Serial.print("k5: "); Serial.println(k5);
      Serial.print("k6: "); Serial.println(k6);
    }
  } else {
    Serial.println("Invalid data format");
  }
}

void loop() {
  while (SerialBT.available()) {
    char receivedChar = SerialBT.read(); // Read a single character
    
    if (receivedChar == '\n') { // Check if it's the end of the message
      parseData(myString); // Parse the received message
      myString = ""; // Reset the string for the next message
    } else {
      myString += receivedChar; // Accumulate characters until newline
    }
  }
  readMPU();
  if((micros() - prevTime_T1) > 6000) {//Set time loop update and control motor
    theta = gettheta(leftencoder,rightencoder)*ToRad; //Read theta value and convert to Rad
    psi = (mpudata)*ToRad;                      //Read psi value and convert to Rad
    phi =  getphi(leftencoder, rightencoder)*ToRad;    //Read phi value and convert to Rad

    //Update time compare with timeloop
    float dt = (float)(micros() - prevTime_T1)/1000000.0;
    prevTime_T1 = micros();
    //Update input angle value
    thetadot = (theta - thetaold)/dt;
    psidot = (psi)/dt;
    phidot = (phi - phiold)/dt;
    //Upadte old angle value
    thetaold = theta;
    psiold = psi;
    phiold = phi;
    //
    addtheta = addtheta + ForwardBack*factortheta;
    addphi = addphi + LeftRight*factorphi;
    
    getlqr(theta + addtheta, thetadot, psi, psidot, phi + addphi, phidot);
    motorControl(PWML, PWMR,(mpudata), falldown);
    String S = "";
    // S = (String)(psi*ToDeg) + ',' + (String)(theta*ToDeg) + ',' + (String)(phi*ToDeg) + ',' + (String)(-addtheta*ToDeg) + ',' + (String)(-addphi*ToDeg)+ ','+ (String)(PWML) + ',' + (String)(PWMR)+','+ (String)(KalmanAnglePitch) + ',' + (String)(k3);
    // S = (String)(psi*ToDeg) + ',' + (String)(theta*ToDeg) + ',' + (String)(phi*ToDeg) + ',' + (String)(-addtheta*ToDeg) + ',' + (String)(-addphi*ToDeg);
    S = (String)(k1) + ',' + (String)(k2) + ',' + (String)(k3) + ',' + (String)(k4) + ',' + (String)(psi*ToDeg) + ',' + (String)(PWML) + ',' + (String)(PWMR);
    SerialBT.println(S);
    // Serial.println(S);
  }
  // motorControl(100, 100,(0 + 0), false);
  
}

