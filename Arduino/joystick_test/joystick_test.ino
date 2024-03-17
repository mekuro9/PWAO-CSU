int linearPin = A0;   // select the input pin for the potentiometer
int turnPin = A1;
int centerPin = A2;

int linear = 0;  // variable to store the value coming from the sensor
int turn = 0;
int center = 0;

void setup() {
 
  Serial.begin(115200);
}

void loop() {
  // read the value from the sensor:
  linear = analogRead(linearPin);
  turn = analogRead(turnPin);
  center = analogRead(center);
  Serial.print("Pin 1: ");
  Serial.print(linear);
  Serial.print(", Pin 2: ");
  Serial.print(turn);
  Serial.print(", Pin 3: ");
  Serial.print(center);
  Serial.println("");
  delay(1000);
}

/* 
voltage 12.08V

Pin1 center = 390
Pin1 max = 470
Pin1 min = 310

Pin2 center = 386/387
Pin2 min = 308
Pin2 max = 466

Pin3 center = 398 /0 
Pin3 max = 
Pin3 min = 
Pin 3 values follow pin1 alternating with 0
*/