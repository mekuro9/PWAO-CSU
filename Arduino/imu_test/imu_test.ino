/*==============================================================
// Filename :       imu_test.ino
// Authors :        Mrinal Magar (s2689529)
// Version :        v1.0
// License :  
// Description :    Testing the imu_utility function library
//                  based on on the driver by ShiromMakkad on Github: 
//                  https://github.com/ShiromMakkad/BNO055
// Tested on Teensy 4.0
==============================================================*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "imu_util.h"

bool zero = true;
bool calibrate = true;
int zeroTime = 50;

double degToRad = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(11);

void setup() {
  //remove serial.begin 
    Serial.begin(115200);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

  if(!bno.begin()){
    // Send imu_status as not connected
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if(calibrate) {
    long bnoID;
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    bool foundCalib = imu_calibrate_check(&sensor, &bno, &bnoID);

    if(!foundCalib){
      //Send sensor status as not calibrated
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
    }
    else{
      // retrieve calibration data
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      imu_retrieve_offsets(&calibrationData, &bno);
      /*Serial.print("Accelerometer: ");
      Serial.print(calibrationData.accel_offset_x); Serial.print(" ");
      Serial.print(calibrationData.accel_offset_y); Serial.print(" ");
      Serial.print(calibrationData.accel_offset_z); Serial.print(" ");

      Serial.print("\nGyro: ");
      Serial.print(calibrationData.gyro_offset_x); Serial.print(" ");
      Serial.print(calibrationData.gyro_offset_y); Serial.print(" ");
      Serial.print(calibrationData.gyro_offset_z); Serial.print(" ");

      Serial.print("\nMag: ");
      Serial.print(calibrationData.mag_offset_x); Serial.print(" ");
      Serial.print(calibrationData.mag_offset_y); Serial.print(" ");
      Serial.print(calibrationData.mag_offset_z); Serial.print(" ");

      Serial.print("\nAccel Radius: ");
      Serial.print(calibrationData.accel_radius);

      Serial.print("\nMag Radius: ");
      Serial.print(calibrationData.mag_radius);*/
    }

    delay(1000);

    //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;

    if(foundCalib){
      // set sensor status as calibrated
      while(!bno.isFullyCalibrated())
      {
        Serial.println("Move sensor slightly to calibrate magnetometers");
        imu_calibrate(&bno, &event);
      }

    }
    else
    {
      while(!bno.isFullyCalibrated())
      {
        Serial.println("Please Calibrate Sensor: ");
        imu_calibrate(&bno, &event);
      }
    }

    adafruit_bno055_offsets_t newCalib;
    imu_store_offsets(&bno, &newCalib, &sensor, &bnoID);
    Serial.println("Data stored to EEPROM.");


  }
  else {
    bno.setExtCrystalUse(true);
  }

  if(zero) {
      // set imu status to zeroing
       Serial.println("Zeroing... Please do not move the device");
      delay(1000);
    }
    
  bno.setMode(OPERATION_MODE_NDOF);

  delay(500);
}
int i = 0;

double totEulerX = 0;
double totEulerY = 0;
double totEulerZ = 0;

double subEulerX = 0;
double subEulerY = 0;
double subEulerZ = 0;

void loop() {
  // put your main code here, to run repeatedly:
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getQuat().toEuler();

    if(zero == false) {
      i = zeroTime + 1;
    }
    if(i < zeroTime) {
      totEulerX += euler.x();
      totEulerY += euler.y();
      totEulerZ += euler.z();
    }
    else if(i == zeroTime) {
      subEulerX = totEulerX / 100;
      subEulerY = totEulerY / 100;
      subEulerZ = totEulerZ / 100;
    }
    else {
      //Display the orientation data
      double x = (euler.y() - subEulerY) * degToRad;
      double y = (euler.z() - subEulerZ) * degToRad;
      double z = (euler.x() - subEulerX) * degToRad;

      Serial.print(F("Orientation: "));
      Serial.print((float)x);
      Serial.print(F(" "));
      Serial.print((float)y);
      Serial.print(F(" "));
      Serial.print((float)z);
      Serial.println(F(""));

      uint8_t sys, gyro, accel, mag = 0;
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      
      Serial.print(F("Calibration: "));
      Serial.print(sys, DEC);
      Serial.print(F(" "));
      Serial.print(gyro, DEC);
      Serial.print(F(" "));
      Serial.print(accel, DEC);
      Serial.print(F(" "));
      Serial.print(mag, DEC);

}
  i += 1;
    /* Wait the specified delay before requesting new data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

