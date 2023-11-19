#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>


char keyinput = '0'; // for incoming serial data
double rotateby = 0; // amount of deg that you want to rotate
double desiredAngle = 0;
double error = 0;

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
  else if(rotateby < -180){
    angle += 360 ;
  }
  else{
    angle += 0 ;
  }  
 return angle;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps 
                      // TODO: Choose a proper baud rate

}

void loop() {
  // put your main code here, to run repeatedly:
   // send data only when you receive data:
  if(Serial.available() > 0 ) {
    //read the incoming key
    keyinput = Serial.read();
    
    Serial.println(keyinput);

    rotateby = keyboardInput(keyinput);

    Serial.println(rotateby);
    
    desiredAngle = wrapAngle(rotateby);
    
    Serial.println(desiredAngle);
  }
    
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
    
