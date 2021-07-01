/**
 * 12.07.2020: Teensy 4.1 as slave
 *    - nRF24L01 Slave https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123/2
 *       - ctrlData is the data to control the servos
 *       - imuData is the data received from the IMUs
 *    - Servo Control
 *    - 10 DOF IMU
 */

/**
 * Standard Libraries
 */
/**
 * Teensy 4.1 Radio Slave
 * CE 10; CSN 9; SCK 13; MISO 12; MOSI 11;
 */
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 10
#define CSN_PIN 9

const byte address[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);
char ctrlData[32] = {0};   
//char imuData_string[32] = {0};                                                       // this must match dataToSend in the TX
float imuData[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // the two values to be sent to the master
bool newData = false;
/*******************************************************************/

/*******************************************************************/
/**
 * Waveshare 10 DOF IMU (Lib: Bolderflight I2C)
 */
#include <MPU9250.h>
MPU9250 IMU_Left(Wire, 0x68);   // "Wire" object refers to pins 18/19
MPU9250 IMU_Right(Wire1, 0x68); // "Wire1" Object refers to pins 16/17 and needs to be initialized in Setup
int status;
/*******************************************************************/

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
char *joystickID, *cRightX, *cRightY, *cLeftX, *cLeftY;
int rightX = 0, rightY = 0, leftX = 0, leftY = 0;
int offsetElevator = 84;
int offsetRudder = 85;
int offsetLeftAileron = 80;
int offsetRightAileron = 100;
int offsetBLDC = 90;
/*******************************************************************/

void setup()
{
  /*******************************************************************/
  Serial.begin(115200);

  // Radio Communication (nRF24L01+LA+PNA)
  Serial.println("SimpleRxAckPayload Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.openReadingPipe(1, address);
  radio.startListening();
  radio.writeAckPayload(1, &imuData, sizeof(imuData)); // pre-load data

  /*******************************************************************/
  // 10 DOF IMU Initialization
  Serial.println("Initializing IMUs...");
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  status = IMU_Left.begin();
  if (status < 0)
  {
    Serial.println("Left IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }
  status = IMU_Right.begin();
  if (status < 0)
  {
    Serial.println("Right IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }
  IMU_Left.setAccelRange(MPU9250::ACCEL_RANGE_8G);          // setting the accelerometer full scale range to +/-8G
  IMU_Left.setGyroRange(MPU9250::GYRO_RANGE_250DPS);        // setting the gyroscope full scale range to +/-250 deg/s
  IMU_Left.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);  // setting DLPF bandwidth to 20 Hz
  IMU_Left.setSrd(19);                                      // setting SRD to 19 for a 50 Hz update rate
  IMU_Right.setAccelRange(MPU9250::ACCEL_RANGE_8G);         // setting the accelerometer full scale range to +/-8G
  IMU_Right.setGyroRange(MPU9250::GYRO_RANGE_250DPS);       // setting the gyroscope full scale range to +/-250 deg/s
  IMU_Right.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ); // setting DLPF bandwidth to 20 Hz
  IMU_Right.setSrd(19);                                     // setting SRD to 19 for a 50 Hz update rate
  Serial.println("IMUs Initialized!");

  /*******************************************************************/
  //Servo Initialization (NodeMCU)
  Serial.println("Initializing Servos...");
  servoElevator.attach(5);
  servoElevator.write(offsetElevator);
  delay(200);
  servoRudder.attach(6);
  servoRudder.write(offsetRudder);
  delay(200);
  servoBLDC.attach(2);
  servoLeftAileron.attach(3);
  servoLeftAileron.write(offsetLeftAileron);
  delay(200);
  servoRightAileron.attach(4);
  servoRightAileron.write(offsetRightAileron);
  delay(200);
  servoBLDC.write(0);
  Serial.println("Servos Initialized!");
  /*******************************************************************/
}

/*******************************************************************/
void updateReplyData()
{
  IMU_Left.readSensor();
  IMU_Right.readSensor();
  imuData[0] = IMU_Left.getAccelX_mss(); // Convert the variable to char
  imuData[1] = IMU_Right.getAccelX_mss();
  imuData[2] = IMU_Left.getAccelY_mss();
  imuData[3] = IMU_Right.getAccelY_mss();
  imuData[4] = IMU_Left.getAccelZ_mss();
  imuData[5] = IMU_Right.getAccelZ_mss();

  imuData[6] = IMU_Left.getGyroX_rads();
  imuData[7] = IMU_Right.getGyroX_rads();
  imuData[8] = IMU_Left.getGyroY_rads();
  imuData[9] = IMU_Right.getGyroY_rads();
  imuData[10] = IMU_Left.getGyroZ_rads();
  imuData[11] = IMU_Right.getGyroZ_rads();

  imuData[12] = IMU_Left.getMagX_uT();
  imuData[13] = IMU_Right.getMagX_uT();
  imuData[14] = IMU_Left.getMagY_uT();
  imuData[15] = IMU_Right.getMagY_uT();
  imuData[16] = IMU_Left.getMagZ_uT();
  imuData[17] = IMU_Right.getMagZ_uT();

  imuData[18] = IMU_Left.getTemperature_C();
  imuData[19] = IMU_Right.getTemperature_C();
  radio.writeAckPayload(1, &imuData, sizeof(imuData)); // load the payload for the next time
}
/*******************************************************************/

void servoActuation()
{
  if (newData == true)
  {
    //Serial.print("Data received ");
    //Serial.println(ctrlData);
    //Serial.print(" ackPayload sent ");
    Serial.print(imuData[0]);
    Serial.print(", ");
    Serial.print(imuData[1]);
    Serial.print(", ");
    Serial.print(imuData[2]);
    Serial.print(", ");
    Serial.print(imuData[3]);
    Serial.print(", ");
    Serial.print(imuData[4]);
    Serial.print(", ");
    Serial.print(imuData[5]);
    Serial.print(", ");
    Serial.print(imuData[6]);
    Serial.print(", ");
    Serial.print(imuData[7]);
    Serial.print(", ");
    Serial.print(imuData[8]);
    Serial.print(", ");
    Serial.print(imuData[9]);
    Serial.print(", ");
    Serial.print(imuData[10]);
    Serial.print(", ");
    Serial.print(imuData[11]);
    Serial.print(", ");
    Serial.print(imuData[12]);
    Serial.print(", ");
    Serial.print(imuData[13]);
    Serial.print(", ");
    Serial.print(imuData[14]);
    Serial.print(", ");
    Serial.print(imuData[15]);
    Serial.print(", ");
    Serial.print(imuData[16]);
    Serial.print(", ");
    Serial.print(imuData[17]);
    Serial.print(", ");
    Serial.print(imuData[18]);
    Serial.print(", ");
    Serial.println(imuData[19]);
    servoElevator.write(rightY + offsetElevator);
    servoRudder.write(leftX + offsetRudder);
    servoLeftAileron.write(rightX + offsetLeftAileron);
    servoRightAileron.write(rightX + offsetRightAileron);
    servoBLDC.write(offsetBLDC - leftY);
    newData = false;
  }
}
/*******************************************************************/

void getData()
{
  if (radio.available())
  {
    radio.read(&ctrlData, sizeof(ctrlData));
    joystickID = strtok((char *)ctrlData, ";");
    if (*joystickID == 'R')
    {
      cRightX = strtok(NULL, ";");
      rightX = atoi(cRightX);
      cRightY = strtok(NULL, ";");
      rightY = atoi(cRightY);
      /*Serial.print("Right Joystick:   ");
          Serial.print(atof(cRightX));
          Serial.print("\t");
          Serial.print(atof(cRightY));
          Serial.print("\n");*/
    }
    else if (*joystickID == 'L')
    {
      cLeftX = strtok(NULL, ";");
      leftX = atoi(cLeftX);
      cLeftY = strtok(NULL, ";");
      leftY = atoi(cLeftY);
      /*Serial.print("Left Joystick:   ");
          Serial.print(atof(cLeftX));
          Serial.print("\t");
          Serial.print(atof(cLeftY));
          Serial.print("\n");*/
    }
    updateReplyData();
    newData = true;
  }
}

//================

void showData()
{
    Serial.print(imuData[0]);
    Serial.print(", ");
    Serial.print(imuData[1]);
    Serial.print(", ");
    Serial.println(imuData[2]);
    /*Serial.print(", ");
    Serial.print(imuData[3]);
    Serial.print(", ");
    Serial.print(imuData[4]);
    Serial.print(", ");
    Serial.print(imuData[5]);
    Serial.print(", ");
    Serial.print(imuData[6]);
    Serial.print(", ");
    Serial.print(imuData[7]);
    Serial.print(", ");
    Serial.print(imuData[8]);
    Serial.print(", ");
    Serial.print(imuData[9]);
    Serial.print(", ");
    Serial.print(imuData[10]);
    Serial.print(", ");
    Serial.print(imuData[11]);
    Serial.print(", ");
    Serial.print(imuData[12]);
    Serial.print(", ");
    Serial.print(imuData[13]);
    Serial.print(", ");
    Serial.print(imuData[14]);
    Serial.print(", ");
    Serial.print(imuData[15]);
    Serial.print(", ");
    Serial.print(imuData[16]);
    Serial.print(", ");
    Serial.print(imuData[17]);
    Serial.print(", ");
    Serial.print(imuData[18]);
    Serial.print(", ");
    Serial.println(imuData[19]);*/
}

//================

void loop()
{
  updateReplyData();
  delay(1000);
  //getData();
  showData();
  //servoActuation();
}
