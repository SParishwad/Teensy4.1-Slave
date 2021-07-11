# Teensy4.1 as a slave for controlling a Model Airplane

This code is a part of a larger project. The Teensy4.1 microcontroller acts as a slave, located in the model airplane and control the Ailerons, Rudder and Elevator servos and the BLDC motor connected to the propellor. 

Another task of the Teensy4.1 microconroller is to process the signals received from the two 10 DOF IMU Modules from [Waveshare](https://www.waveshare.com/wiki/10_DOF_IMU_Sensor_(C)) and close the feedback loop to  stabilize the plane during the flight. 

The Code can be split into the following parts:

1. Radio Communication with the Master Module
2. Servo Control
3. IMU Data processing, control of the plane and transmission of the Roll, pitch and Yaw values to the Master module via Radio

## Radio Communication with the Master Module

The nRF24L01+LA+PNA module is used to communicate between the Plane and the transmitter on the ground. Theoretically it provides upto 800m of range.

The RF24 Library by [tmrh20](https://github.com/nRF24/RF24.git) is used. A "[Two-way transmission using the ackPayload concept](https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123/2)" concept is used to receive the control signals from the NodeMCU Master Module and transmit the roll, pitch and yaw values through the acknowledgement. 

```
#include <nRF24L01.h>

void setup(){
  Serial.println("Radio Communnication Starting...");
  radio.begin();
  radio.openReadingPipe(0, address); // 00001
  radio.setPALevel(RF24_PA_MIN);
  delay(1000);
  Serial.println("Radio Communication Started!");
  radio.startListening();
}
```

The data is read from the nRF24L01 buffer by using the command `radio.read(&dataReceived, sizeof(dataReceived));` and the acknowledgement is sent using the command `radio.write(&ackData, sizeof(ackData));`. It is essential to note, that the ackData should be ready before the *radio.read* function is executed. 

** See the IMU-Radio_Communication branch for the transmission of IMU-Data to the master module. SHall be merged with the main branch after further testing. 


## Servo Control 
The Ailerons, Elevator and Rudder are controlled via 4 servos incorporated in the model airplane design. The BLDC can also be controlled by by using the servo library. 

The library is included and the servo objects are created. In the setup section the servos are initialized and centered. In the main section, the `servo_obj.write()` function is used to control the servos based on the commands received from the NodeMCU Master.   

```
#include <Servo.h>
Servo servoElevator;
Servo servoRudder;
Servo servoLeftAileron;
Servo servoRightAileron;
Servo servoBLDC;

void setup(){
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
}

void main(){
    servoElevator.write(rightY + offsetElevator);
    servoRudder.write(leftX + offsetRudder);
    servoLeftAileron.write(rightX + offsetLeftAileron);
    servoRightAileron.write(rightX + offsetRightAileron);
    servoBLDC.write(offsetBLDC - leftY);
}
```

## IMU Data processing, control of the plane and transmission of the Roll, pitch and Yaw values to the Master module via Radio

The 10 DOF IMU sensor from Waveshare contains two mems sensors, the MPU9205 which consists of 3-axis gyroscope, 3-axis accelerometer, and 3-axis compass/magnetometer and a BMP280 which is a Barometer pressure sensor and also provides the temperature value. 

The 3-axis gyroscope, accelerometer and magnetometer data can be converted into roll, pitch and yaw using one of the four sensor fusion algorithms. 

* Complementary Filter
* Kalman Filter
* Madwick Filter
* Mahony Filter

However, the data processing of the IMU sensor is a comprehensive topic and merits a separate repository of its own. 

Link: 
