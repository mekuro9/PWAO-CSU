#include "PIDController.hpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include "imu_util.h"
#include <ctime>

bool zero = true;
bool calibrate = true;
int zeroTime = 50;
float degToRad = 57.295779513;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(11);
// Global variable orient 
imu::Vector<3> gyro;
imu::Vector<3> angle;

// Motor control pins for two motors

const int u1F = 5; // u1 forward
const int u1B = 6; // u1 backward
const int u2F = 3; //u2 forward
const int u2B = 4;  // u2 backward

unsigned long startTime;
unsigned long currentTime;

// PID controller setup
float gyroSetpoint, gyroMeasure;
float angleSetpoint, angleMeasure;
float OuterControllerOutput;
int uController;
bool justonce = true;
char keyinput = '0'; //incoming serial data

float rotateby = 0; // amount of degree you want to rotate;

float desiredAngle = 0.0; // Desired turn angle in degrees
float currentAngle = 0.0; // Current angle from IMU
float motorSpeed = 0.0; // Speed correction for motors

//controller type 
pid_controller_t Ocontroller = typePID;
pid_controller_t Icontroller = typePID;
// PID gains - these need tuning for your specific application
float ikp = 2;//0.05;
float iki = 0.005;//0.005;
float ikd = 1;//0;
// PID OuterLoop
float Okp = 5;//10;
float Oki = 0.0005;//0.5;
float Okd = 1;//10;

// PID Controller for motorPID, gyroSetpoint is the OuterControllerOutput which is rad/s. The measurement is angular velocity and the output is int pwm
PIDController<float, int> motorPID(&gyroSetpoint, &gyroMeasure, &uController, ikp, iki, ikd, Icontroller);
// PID Controller for outerLoop, angleSetpoint is the desired Angle which in deg. The measurement is orientation in deg and the output is desired angular velocity in deg/s
PIDController<float,float> OuterLoop(&angleSetpoint, &angleMeasure, &OuterControllerOutput,Okp,Oki,Okd,Ocontroller);
// INCOMING KEYBOARD DATA FSM ///////////
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
////////////////////////////////////////
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
///////////////////////////////////////////


//// UTILITY FOR MOTOR CONTROL //////////
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

float tau = 0;


void setup() {
    stop();
    pinMode(LED_BUILTIN, OUTPUT);
    ///////////////////////////////////////////////
    // imu Calibration
    ///////////////////////////////////////////////
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
        digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
        imu_calibrate(&bno, &event);
      }
      digitalWrite(LED_BUILTIN, LOW);

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
  /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  angle = bno.getQuat().toEuler();
  // Initialize the PID controller
  bool check = motorPID.setup();
  Serial.println(check);
  Serial.println("");
  check = OuterLoop.setup();
  Serial.println(check);
  Serial.println("");
  motorPID.setControllerType(Icontroller);
  OuterLoop.setControllerType(Ocontroller);
  // Set the sample time (in milliseconds)
  motorPID.setSampleTime(100);
  OuterLoop.setSampleTime(100);

  // Set output limits for turning speed
  motorPID.setOutputLimits(-200, 200); // Full speed backward to full speed forward
  OuterLoop.setOutputLimits(-100, 100); // angular velocity limits deg/s

  // Initialize the motor control pins as outputs
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
int mapped_input = 160;
float prevAngle = 0;
float error = 0;
float errorO = 0;

void loop() {

    currentTime = millis();
    tau = (currentTime - startTime);
    if(tau > 10){
        startTime = millis();
        angle = bno.getQuat().toEuler();
        float z = angle.x() * degToRad;
        gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        if(Serial.available() > 0 ) {
        
        keyinput = Serial.read(); //read the incoming key
        Serial.println("Keyinput ");
        Serial.print(keyinput); //print the serial input

        rotateby = keyboardInput(keyinput); // function to rotate
        Serial.println("Rotateby: ");
        Serial.print(rotateby); 
        
        desiredAngle = wrapAngle(rotateby); //function to wrap the angle 
        Serial.println("Desired Angle: ");
        Serial.print(desiredAngle);
        Serial.println("");
      }

    if(justonce == true){
        rotateby = wrapAngle((float)z);
        desiredAngle = wrapAngle((float)z);
        angleMeasure = wrapAngle((float)z);
        angleSetpoint = desiredAngle;
        gyroMeasure = 0.0;//(float)gyro.x();
        
        // Compute the PID controller output
        OuterLoop.compute();
        gyroSetpoint = (float)OuterControllerOutput; //setting gyro setpoint in deg/s
        motorPID.compute();
        justonce = false;
      }

      angleMeasure = wrapAngle((float)z);
      gyroMeasure = (angleMeasure - prevAngle)*1000/tau;//(float)gyro.x();
      angleSetpoint = desiredAngle;

      OuterLoop.compute();
      errorO = OuterLoop.geterror();
      gyroSetpoint = (float)OuterControllerOutput;//errorO; 
      motorPID.compute();
      error = motorPID.geterror();
    
      //if((abs(error) > 2)){
      // Apply the controller output to the motors to turn
      if (error > 0) {
        // Turn left
          mapped_input = map(uController,0,200,160,200);
          turnleft(abs(mapped_input),abs(mapped_input));
        } else {
        // Turn right
          mapped_input = map(uController,-200,0,200,160);
          turnright(abs(mapped_input),abs(mapped_input));
        }
      
            //print values for debugging
      Serial.print("angleSet: ");
      Serial.print(angleSetpoint);
      Serial.print(", Curr ang: ");
      Serial.print(angleMeasure);
      Serial.print(" gyroSet: ");
      Serial.print(gyroSetpoint);
      Serial.print(", Curr gyro: ");
      Serial.print(gyroMeasure);
      Serial.print(", Error: ");
      Serial.print(errorO);
      //Serial.print(", MotorSpe: ");
      //Serial.print(uController);
      Serial.print(", Mapped: ");
      Serial.print(mapped_input);
      Serial.print(", tau: ");
      Serial.print(tau);
      Serial.println("");
      prevAngle = angleMeasure;
    }
    //}


    // Delay for a bit before the next loop iteration

    delay(10);
    
}

