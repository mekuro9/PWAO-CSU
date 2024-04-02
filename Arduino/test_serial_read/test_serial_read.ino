void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

char keyinput;

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0) {
    keyinput = Serial.read();
  }
  Serial.println("lol");
  delay(2000);
  
}
