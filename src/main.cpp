/**
 * 12.07.2020: Teensy 4.1 as slave
 *    - nRF24L01 Slave
 *    - Servo Control
 *    - 10 DOF IMU
 */

/**
 * Standard Libraries
 */
#include <Arduino.h>
#include <SPI.h>
// int led = 13;
/*******************************************************************/

/**
 * Teensy 4.1 Radio Slave
 * CE 10; CSN 9; SCK 13; MISO 12; MOSI 11;
 */
#include "nRF24L01.h"
#include "RF24.h"
#define CE  10
#define CSN 9
const byte masterAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE, CSN);
char dataReceived[10]; // this must match dataToSend in the TX
int ackData[19] = {0, 0}; // This array will be sent to the Master with the IMU Data. I can send upto 32 bytes of data at a time. 
bool newData = false;
/*******************************************************************/

/**
 * Servo Motor Control
 */
#include <Servo.h>
Servo servoElevator; 
Servo servoRudder;
Servo servoLeftAileron;
Servo servoRightAileron;
Servo servoBLDC;
char * joystickID,* cRightX, * cRightY, * cLeftX, * cLeftY;
int rightX = 0, rightY = 0, leftX = 0, leftY = 0;
int offsetElevator = 84;
int offsetRudder = 85; 
int offsetLeftAileron = 80; 
int offsetRightAileron = 100;
int offsetBLDC = 90;
/*******************************************************************/

/**
 * Waveshare 10 DOF IMU (Lib: Bolderflight I2C)
 */
#include "MPU9250.h"
MPU9250 IMU_Left(Wire,0x68);              // "Wire" object refers to pins 18/19
MPU9250 IMU_Right(Wire1,0x68);            // "Wire1" Object refers to pins 16/17 and needs to be initialized in Setup
int status;
/*******************************************************************/

void setup() {
  Serial.begin(115200);
  delay(10);
  // pinMode(led, OUTPUT);
  
  //Radio Communication (nRF24L01+LA+PNA)
  Serial.println("Radio Communnication Starting...");
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3,5);
  radio.openReadingPipe(1, masterAddress);
  radio.enableAckPayload();
  radio.startListening();
  radio.writeAckPayload(1, &ackData, sizeof(ackData)); // pre-load data
  Serial.println("Radio Communication Started!");

  //Servo Initialization (NodeMCU)
  Serial.println("Initializing Servos...");
  servoElevator.attach(5);
  servoElevator.write(offsetElevator);
  delay(200);
  servoRudder.attach(4);
  servoRudder.write(offsetRudder);
  delay(200);
  servoBLDC.attach(14);
  servoLeftAileron.attach(0);
  servoLeftAileron.write(offsetLeftAileron);
  delay(200);
  servoRightAileron.attach(2);
  servoRightAileron.write(offsetRightAileron);
  delay(200);
  servoBLDC.write(0);
  Serial.println("Servos Initialized!");

  // 10 DOF IMU Initialization
  Serial.println("Initializing IMUs...");
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  status = IMU_Left.begin();
  if (status < 0) {
    Serial.println("Left IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  status = IMU_Right.begin();
  if (status < 0) {
    Serial.println("Right IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  IMU_Left.setAccelRange(MPU9250::ACCEL_RANGE_8G);                  // setting the accelerometer full scale range to +/-8G 
  IMU_Left.setGyroRange(MPU9250::GYRO_RANGE_500DPS);                // setting the gyroscope full scale range to +/-500 deg/s
  IMU_Left.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);          // setting DLPF bandwidth to 20 Hz
  IMU_Left.setSrd(19);                                              // setting SRD to 19 for a 50 Hz update rate 
  IMU_Right.setAccelRange(MPU9250::ACCEL_RANGE_8G);                 // setting the accelerometer full scale range to +/-8G
  IMU_Right.setGyroRange(MPU9250::GYRO_RANGE_500DPS);               // setting the gyroscope full scale range to +/-500 deg/s
  IMU_Right.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);         // setting DLPF bandwidth to 20 Hz
  IMU_Right.setSrd(19);                                             // setting SRD to 19 for a 50 Hz update rate
  Serial.println("IMUs Initialized!");

}

void loop() {
  if ( radio.available() ) {
    radio.read( &dataReceived, sizeof(dataReceived) );
    joystickID = strtok((char*)dataReceived, ";");
    if(*joystickID == 'R'){
      cRightX = strtok(NULL, ";");
      rightX = atoi(cRightX);
      cRightY = strtok(NULL, ";");
      rightY = atoi(cRightY);
      // Serial.print("Right Joystick:   ");
      // Serial.print(atof(cRightX));
      // Serial.print("\t");
      // Serial.print(atof(cRightY));
      // Serial.print("\n");
    }
    else if(*joystickID == 'L'){
      cLeftX = strtok(NULL, ";");
      leftX = atoi(cLeftX);
      cLeftY = strtok(NULL, ";");
      leftY = atoi(cLeftY);
    }
    newData = true;
  }

  if(newData == true){
    servoElevator.write(rightY+offsetElevator);
    servoRudder.write(leftX+offsetRudder);
    servoLeftAileron.write(rightX+offsetLeftAileron);
    servoRightAileron.write(rightX+offsetRightAileron);
    servoBLDC.write(offsetBLDC-leftY);
    newData = false;


    IMU_Left.readSensor();
    IMU_Right.readSensor();
    ackData[0] = IMU_Left.getAccelX_mss();   // Convert the variable to char
    ackData[1] = IMU_Right.getAccelX_mss();
    ackData[2] = IMU_Left.getAccelY_mss();
    ackData[3] = IMU_Right.getAccelY_mss();
    ackData[4] = IMU_Left.getAccelZ_mss();
    ackData[5] = IMU_Right.getAccelZ_mss();

    ackData[6] = IMU_Left.getGyroX_rads();
    ackData[7] = IMU_Right.getGyroX_rads();
    ackData[8] = IMU_Left.getGyroY_rads();
    ackData[9] = IMU_Right.getGyroY_rads();
    ackData[10] = IMU_Left.getGyroZ_rads();
    ackData[11] = IMU_Right.getGyroZ_rads();

    ackData[12] = IMU_Left.getMagX_uT();
    ackData[13] = IMU_Right.getMagX_uT();
    ackData[14] = IMU_Left.getMagY_uT();
    ackData[15] = IMU_Right.getMagY_uT();
    ackData[16] = IMU_Left.getMagZ_uT();
    ackData[17] = IMU_Right.getMagZ_uT();

    ackData[18] = IMU_Left.getTemperature_C();
    ackData[19] = IMU_Right.getTemperature_C();
    radio.writeAckPayload(1, &ackData, sizeof(ackData));
  }


}