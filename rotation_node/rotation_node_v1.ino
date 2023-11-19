#include <PID_v1.h>

char key = '0'; // for incoming serial data
double rotateby = 0;
double desiredAngle = 0;
double error = 0;

double setPoint, Input, Output;

double Kp = 2, Ki = 5, Kd = 1; // PID tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the key:
    key = Serial.read();
    keyboardInput(key);
  }

  desiredAngle = wrapAngle(rotateby);

}

//do i have to add rotateby variable to this function input?
void keyboardInput(char input){
// WSAD or wsad keys used as input.
// 'a' or 'A' rotates by -10 degrees
// 'd' or 'D' rotates by +10 degrees (so, in the other direction)
    switch (input) {
  case 'a':
    rotateby += -10;
    //do something when var equals 1
    break;
    case 'A':
    rotateby += -10;
    //do something when var equals 1
    break;
  case 'd':
    rotateby += +10;
    //do something when var equals 2
    break;
  case 'D':
    rotateby += +10;
    //do something when var equals 2
    break;
  default:
    // if nothing else matches, do the default
    rotateby += 0;
    break;    
    
  }

}

double wrapAngle(double angle){
  // Wrap the angle rotation by +- 360 deg
  if(angle > 360){
    angle -= 360;
  }
  else if(angle < -360){
    angle += 360 ;
  }
  else{
    angle += 0 ;
  }  

}

// use PID library to get error
