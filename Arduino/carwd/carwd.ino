//////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "imu_util.h"
#include <ctime>

bool zero = true;
bool calibrate = true;
int zeroTime = 50;

double degToRad = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(11);


const int u1F = 5;
const int u1B = 6;
const int u2F = 3;
const int u2B = 4;

unsigned long startTime;
unsigned long currentTime;


// Forward Backward motion: u1F +u1B + u2F + u2B
// Turing: u2F + u2B - u1F - u2F 
int pwm_array[4] = {0,0,0,0}; // u1f, u2f, u1b, u2b

void stop(){
  analogWrite(3,0);
  pwm_array[0] = 0;
  analogWrite(4,0);
  pwm_array[1] = 0;
  analogWrite(5,0);
  pwm_array[2] = 0;
  analogWrite(6,0);
  pwm_array[3] = 0;


}

void goforward(int speed1, int speed2){

  analogWrite(u1F,speed1);
  pwm_array[0] = speed1;
  analogWrite(u2F,speed2);
  pwm_array[1] = speed2;
  analogWrite(u1B,0);
  pwm_array[2] = 0;
  analogWrite(u2B,0);
  pwm_array[3] = 0;

}

void turnleft(int speed1, int speed2){
  analogWrite(u1F,0);
  pwm_array[0] = 0;
  analogWrite(u2F,speed2);
  pwm_array[1] = speed2;
  analogWrite(u1B,speed1);
  pwm_array[2] = speed1;
  analogWrite(u2B,0);
  pwm_array[3] = 0;
}

void turnright(int speed1, int speed2){
  analogWrite(u1F,speed1);
  pwm_array[0] = speed1;
  analogWrite(u2F,0);
  pwm_array[1] = 0;
  analogWrite(u1B,0);
  pwm_array[2] = 0;
  analogWrite(u2B,speed2);
  pwm_array[3] = speed2;
}


void gobackward(int speed1, int speed2){

  analogWrite(u1F,0);
  pwm_array[0] = 0;
  analogWrite(u2F,0);
  pwm_array[1] = 0;
  analogWrite(u1B,speed1);
  pwm_array[2] = speed1;
  analogWrite(u2B,speed2);
  pwm_array[3] = speed2;

}

void setup() {

  // imu
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




  pinMode(u1F, OUTPUT);
  pinMode(u1B, OUTPUT);
  pinMode(u2F, OUTPUT);
  pinMode(u2B, OUTPUT);

  stop();
  Serial.println("Starting measurement...");
  startTime = millis();

}



bool state = true;
int loop_ctr = 0;
int max_speed = 255;
int min_speed = 160;
int pwm_ctr = min_speed;

const int array_size = 50;
float lin_a_z[array_size];
float lin_a_y[array_size];
float gyro_x[array_size];
float orient_x[array_size];
int pwm1_f[array_size];
int pwm1_b[array_size];
int pwm2_f[array_size];
int pwm2_b[array_size];

unsigned long measurementTime[array_size];

void loop() {

    
  // put your main code here, to run repeatedly:
    currentTime = millis();
    if(currentTime - startTime > 100 && loop_ctr < array_size){
      startTime = millis();

      measurementTime[loop_ctr] = startTime;

      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      orient_x[loop_ctr] = euler.x();
      gyro_x[loop_ctr] = gyro.x();
      lin_a_y[loop_ctr] = linear.y();
      lin_a_z[loop_ctr] = linear.z();

      if (loop_ctr <= 10) {
        stop();// u1f, u2f, u1b, u2b

      } else {

        turnright(190,170);

      }

        pwm1_f[loop_ctr] = pwm_array[0];
        pwm1_b[loop_ctr] = pwm_array[2];
        pwm2_f[loop_ctr] = pwm_array[1];
        pwm2_b[loop_ctr] = pwm_array[3];
      

        if(loop_ctr == array_size -1 ){
          Serial.print("\n");
          Serial.print("Time: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(measurementTime[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("Orient: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(orient_x[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("lin_a_z: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(lin_a_z[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("lin_a_y: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(lin_a_y[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("gyro_x: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(gyro_x[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("1_f: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(pwm1_f[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("1_b: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(pwm1_b[i]);
            Serial.print(", ");
          }
          Serial.print("\n");

          Serial.print("2_f: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(pwm2_f[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
          Serial.print("2_b: ");
          for (int i = 0; i<array_size; i++) {     
            Serial.print(pwm2_b[i]);
            Serial.print(", ");
          }
          Serial.print("\n");
    }

    loop_ctr++;

    } else if (loop_ctr >= array_size) {
      stop();
    }
    


  

}


