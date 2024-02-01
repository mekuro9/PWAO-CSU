/*==============================================================
// Filename :       imu_util.cpp
// Authors :        Mrinal Magar 
// Version :        v1.0
// License :  
// Description :    Utility functions for imu bno055 for
//                  calibration of data and storing in teensy eeprom,
//                  retrieving data from eeprom, and updating the
//                  offsets in eeprom after every interval
// Tested on Teensy 4.0
==============================================================*/


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "imu_util.h"


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)



/* Function looks for sensor's unique ID at the beginning of EEPROM */
bool imu_calibrate_check(sensor_t* sensor, Adafruit_BNO055* bno, long* bnoID){
    EEPROM.get(0, *bnoID);
    bno->getSensor(sensor);
    if(*bnoID != sensor->sensor_id){
        return false; // return false of check fails
    }
    else{
        return true; // return true if check succeeds
    }

}

/*Function retrieves offset data which is stored in EEPROM of teensy*/
void imu_retrieve_offsets(adafruit_bno055_offsets_t* calibrationData, Adafruit_BNO055* bno){ 
    int eeAddress = 0;
    eeAddress += sizeof(long);
    //storing the offset data from EEPROM to calibrationData 
    EEPROM.get(eeAddress,*calibrationData);
    // set sensor offsets
    bno->setSensorOffsets(*calibrationData); 
}

/*Waits for the imu to be calibrated*/
void imu_calibrate(Adafruit_BNO055* bno, sensors_event_t* event){
    while(!bno->isFullyCalibrated())
    {
        bno->getEvent(event);
       
       /* imu::Vector<3> euler = bno->getQuat().toEuler();
              
              double x = euler.y() * 57.295779513;
              double y = euler.z() * 57.295779513;
              double z = euler.x() * 57.295779513;
              
              Serial.print("X: ");
              Serial.print(x, 4);
              Serial.print(" Y: ");
              Serial.print(y, 4);
              Serial.print(" Z: ");
              Serial.print(z, 4);
              Serial.println("\t\t"); //  For Debugging*/ 

        delay(BNO055_SAMPLERATE_DELAY_MS);
    }

}

/* Store offsets in EEPROM*/
void imu_store_offsets(Adafruit_BNO055* bno, adafruit_bno055_offsets_t* calibrationData, sensor_t* sensor, long* bnoID){
    // Get offsets from the sensor
    bno->getSensorOffsets(*calibrationData);
    int eeAddress = 0;
    bno->getSensor(sensor);
    // get the sensor id
    *bnoID = sensor->sensor_id;
    // store the sensor id in EEPROM
    EEPROM.put(eeAddress, *bnoID);
    eeAddress += sizeof(long);
    // store the sensor calibration data
    EEPROM.put(eeAddress, *calibrationData);
}
