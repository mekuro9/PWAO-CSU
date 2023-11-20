#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

bool zero = true;
bool calibrate = true;
int zeroTime = 50;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

double degToRad = 57.295779513;

Adafruit_BNO055 bno = Adafruit_BNO055(6969);

// Global variable orient 
imu::Vector<3> orient;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void bunnyPrintOrientation(double x, double y, double z) {
  Serial.print(F("Orientation: "));
  Serial.print((float)x);
  Serial.print(F(" "));
  Serial.print((float)y);
  Serial.print(F(" "));
  Serial.print((float)z);
  Serial.println(F(""));
}

void bunnyPrintCalibration() {
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

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
bool justonce = true; // used for initialization of the setpoint

char keyinput = '0'; // for incoming serial data
double rotateby = 0; // amount of deg that you want to rotate
double desiredAngle = 0;
double error = 0;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


double keyboardInput(char input){

  if((input == 'a')or(input == 'A')){
        rotateby -= 10;
  }
  else if((input == 'd')or(input == 'D')){
    rotateby += 10;
  }
  else{
    rotateby += 0;
  }

  return rotateby;

}
 
double wrapAngle(double angle){
  // Wrap the angle rotation by +- 180 deg
  if(angle > 180){
    angle -= 360;
  }
  else if(angle < -180){
    angle += 360 ;
  }
  else{
    angle += 0 ;
  }  
 return angle;
}


void setup() {

  /**************************************************************************/
/*
    Setup for calibration of IMU
    */
/**************************************************************************/

    Serial.begin(115200); // opens serial port, sets data rate to 9600 bps 
                      // TODO: Choose a proper baud rate
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    if(calibrate) {
      int eeAddress = 0;
      long bnoID;
      bool foundCalib = false;
  
      EEPROM.get(eeAddress, bnoID);
  
      adafruit_bno055_offsets_t calibrationData;
      sensor_t sensor;
  
      /*
      *  Look for the sensor's unique ID at the beginning oF EEPROM.
      *  This isn't foolproof, but it's better than nothing.
      */
      bno.getSensor(&sensor);
      if (bnoID != sensor.sensor_id)
      {
          Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
          delay(500);
      }
      else
      {
          Serial.println("\nFound Calibration for this sensor in EEPROM.");
          eeAddress += sizeof(long);
          EEPROM.get(eeAddress, calibrationData);
  
          displaySensorOffsets(calibrationData);
  
          Serial.println("\n\nRestoring Calibration data to the BNO055...");
          bno.setSensorOffsets(calibrationData);
  
          Serial.println("\n\nCalibration data loaded into BNO055");
          foundCalib = true;
      }
  
      delay(1000);
  
      /* Display some basic information on this sensor */
      displaySensorDetails();
  
      /* Optional: Display current status */
      displaySensorStatus();
  
     //Crystal must be configured AFTER loading calibration data into BNO055.
      bno.setExtCrystalUse(true);
  
      sensors_event_t event;
      bno.getEvent(&event);
      if (foundCalib){
          Serial.println("Move sensor slightly to calibrate magnetometers");
          while (!bno.isFullyCalibrated())
          {
              bno.getEvent(&event);
              delay(BNO055_SAMPLERATE_DELAY_MS);
          }
      }
      else
      {
          Serial.println("Please Calibrate Sensor: ");
          while (!bno.isFullyCalibrated())
          {
              bno.getEvent(&event);
  
              imu::Vector<3> euler = bno.getQuat().toEuler();
              
              double x = euler.y() * degToRad;
              double y = euler.z() * degToRad;
              double z = euler.x() * degToRad;
              
              Serial.print("X: ");
              Serial.print(x, 4);
              Serial.print(" Y: ");
              Serial.print(y, 4);
              Serial.print(" Z: ");
              Serial.print(z, 4);
              Serial.print("\t\t");
  
              /* Optional: Display calibration status */
              displayCalStatus();
  
              /* New line for the next sample */
              Serial.println("");
  
              /* Wait the specified delay before requesting new data */
              delay(BNO055_SAMPLERATE_DELAY_MS);
          }
      }
  
      Serial.println("\nFully calibrated!");
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");
      adafruit_bno055_offsets_t newCalib;
      bno.getSensorOffsets(newCalib);
      displaySensorOffsets(newCalib);
  
      Serial.println("\n\nStoring calibration data to EEPROM...");
  
      eeAddress = 0;
      bno.getSensor(&sensor);
      bnoID = sensor.sensor_id;
  
      EEPROM.put(eeAddress, bnoID);
  
      eeAddress += sizeof(long);
      EEPROM.put(eeAddress, newCalib);
      Serial.println("Data stored to EEPROM.");
    }
    else {
      bno.setExtCrystalUse(true);
    }
    
    if(zero) {
      Serial.println("Zeroing... Please do not move the device");
      delay(1000);
    }
    
    bno.setMode(0X0C);

    delay(500);

      /**************************************************************************/
/*
    Setup for PID
    */
/**************************************************************************/
    orient = bno.getQuat().toEuler();
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

int i = 0;

double totEulerX = 0;
double totEulerY = 0;
double totEulerZ = 0;

double subEulerX = 0;
double subEulerY = 0;
double subEulerZ = 0;

void loop() {

  /**************************************************************************/
/*
    Getting keyboard data
    */
/**************************************************************************/
  // send data only when you receive data:
  if(Serial.available() > 0 ) {
    
    keyinput = Serial.read(); //read the incoming key
    
    Serial.println(keyinput); //print the serial input

    rotateby = keyboardInput(keyinput); // function to rotate

    Serial.println(rotateby); 
    
    desiredAngle = wrapAngle(rotateby); //function to wrap the angle 
    
    Serial.println(desiredAngle);
  }

  
  /**************************************************************************/
/*
    IMU orientation data
    */
/**************************************************************************/

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
      
      double x = (euler.y() - subEulerY)* degToRad;
      double y = (euler.z() - subEulerZ)* degToRad;
      double z = (euler.x() - subEulerX)* degToRad;
    
    if(justonce == true){
      rotateby = wrapAngle(z);
      desiredAngle = wrapAngle(z);
      justonce = false;
    }
      //PID input
      Input = wrapAngle(z);
      
      
      //Optional: Display calibration status
      //displayCalStatus();
  
      //Optional: Display sensor status (debug only)
      //displaySensorStatus();
      
    }

      /**************************************************************************/
/*
    PID controller data
    */
/**************************************************************************/
      Setpoint = desiredAngle;
      myPID.Compute();
      
      error = Input - Setpoint;
      Serial.print("error: ");
      Serial.print(error);
       Serial.print("desiredAngle: ");
      Serial.print(desiredAngle);
       Serial.print("Current Input: ");
      Serial.print(Input);
      /* New line for the next sample */
      Serial.println("");
  


    /* Wait the specified delay before requesting new data */
    i += 1;
    delay(BNO055_SAMPLERATE_DELAY_MS);
}

static int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

// Steps to do
//1. Implement imu calibration
//2. Implement reading imu data
//3. Compare current reading to user input +- 10 deg
//4. Generate error signal and use PID control generate joystick like signal
//5. output the current (rot) reading to ros2 via serial or other way 
    
    // I have to add the imu calibration (initally calibrate the imu if not already done or calibrate it if too much variation)
    // Do i have to keep calibrating imu after certain amount of time?? what if it is not physically possible?
    //
    // can i read keyboard data from ros2 and send it to arduino?? not needed perhaps?
    
