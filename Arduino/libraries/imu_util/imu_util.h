/*==============================================================
// Filename :       imu_util.cpp
// Authors :        Mrinal Magar (s2689529)
// Version :        v1.0
// License :  
// Description :    Utility functions for imu bno055 for
//                  calibration of data and storing in teensy eeprom,
//                  retrieving data from eeprom, and updating the
//                  offsets in eeprom after every interval
==============================================================*/

#ifndef imu_util_h
#define imu_util_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*
This library is based on the driver by ShiromMakkad on Github: https://github.com/ShiromMakkad/BNO055

This utility header file has function declarations.
The function definitions are in the imu_utility.cpp file

*/
bool imu_calibrate_check(sensor_t* sensor, Adafruit_BNO055* bno, long* bnoID); // Check if calibration data is stored in eeprom

void imu_calibrate(Adafruit_BNO055* bno, sensors_event_t* event);// Calibrate the imu if not done so

void imu_retrieve_offsets(adafruit_bno055_offsets_t* calibrationData, Adafruit_BNO055* bno); // Retrieve offset data from eeprom
   
void imu_store_offsets(Adafruit_BNO055* bno, adafruit_bno055_offsets_t* calibrationData, sensor_t* sensor, long* bnoID); // stores offset data in eeprom

#endif