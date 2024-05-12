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
#define BNO055_SAMPLERATE_DELAY_MS (10)
//extern TwoWire Wire2;

Adafruit_BNO055 bno = Adafruit_BNO055(11,0x28,&Wire2);

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
      Serial.print("Accelerometer: ");
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
      Serial.print(calibrationData.mag_radius);
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

imu::Vector<3> orient;
float previousAngle = 0.0;
int completeRotations = 0;

float getUnwrappedAngle(float newAngle) {
    float delta = newAngle - previousAngle;

    if (delta > 180) {
        // Counter-clockwise crossing 0
        completeRotations -= 1;
    } else if (delta < -180) {
        // Clockwise crossing 360
        completeRotations += 1;
    }

    previousAngle = newAngle;
    return newAngle + completeRotations * 360;
}
void loop() {
  // put your main code here, to run repeatedly:
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getQuat().toEuler();
    float currentAngle = getUnwrappedAngle(euler.x()*degToRad);

      Serial.print(F("Orientation: "));
      Serial.print((float)currentAngle);
      Serial.println(F(""));

   
    /* Wait the specified delay before requesting new data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

