// generate a voltage through a digital potentiometer
// center voltage should be 6V
// swing +/- 1 V

#include <Wire.h>

const int shutDownPin = 2;

int linearPin = A0;   // select the input pin for the potentiometer
int turnPin = A1;
int centerPin = A2;

int linear = 0;  // variable to store the value coming from the sensor
int turn = 0;
int center = 0;


void setup() {
  // put your setup code here, to run once:
  // set the shutDownPin as an output
  pinMode(shutDownPin, OUTPUT);
  //Shutdown the pots to start with
  digitalWrite(shutDownPin, HIGH);
  Serial.begin(115200);
  Wire.begin();

}
int val = 256;
//val = map(val, 0, 1023, 0, 255);
void loop() {
  linear = analogRead(linearPin);
  turn = analogRead(turnPin);
  center = analogRead(center);
  // this if else logic only checks if the joystick is connected or not because analog read outputs 0 if nothing attached
if(linear > 300 && turn > 300){
  int pot1 = map(linear, 300, 470, 55, 255);//map(linear, 300, 470, 105, 255); //17
  int pot2 = map(turn, 300, 470, 87, 255);//map(turn,300, 470, 92, 255); //24

  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(pot1);
  int error = Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(pot2);
  Wire.endTransmission();
  Serial.print("Pin 1: ");
  Serial.print(linear);
  Serial.print(", Pin 2: ");
  Serial.print(turn);
  Serial.print(", Pot 1: ");
  Serial.print(pot1);
  Serial.print(", Pot 2: ");
  Serial.print(pot2);
  Serial.println("");}
  else{

  int pot1 = 156;//177;//129;// 5.924
  int pot2 = 165;//128;// 5.920

  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // the other one: 0x00, 0x80
  Wire.write(pot1);
  int error = Wire.endTransmission();
  Wire.beginTransmission(0x2C);
  Wire.write(0x80);  // the other one: 0x00, 0x80
  Wire.write(pot2);
  Wire.endTransmission();
  Serial.print("Else ");
  Serial.print("Pin 1: ");
  Serial.print(linear);
  Serial.print(", Pin 2: ");
  Serial.print(turn);
  Serial.print(", Pin 3: ");
  Serial.print(", Pot 1: ");
  Serial.print(pot1);
  Serial.print(", Pot 2: ");
  Serial.print(pot2);
  Serial.println("");
  }
  
  delay(100);
  
}

//mid

/*
177, 165 -> works (full charged)
*/
