int pwm_ctr = 0;

const int u1F = 5;
const int u1B = 6;
const int u2F = 3;
const int u2B = 4;

// Forward Backward motion: u1F +u1B + u2F + u2B
// Turing: u2F + u2B - u1F - u2F 
void stop(){
  analogWrite(3,0);
  analogWrite(4,0);
  analogWrite(5,0);
  analogWrite(6,0);

}

void goforward(int speed1, int speed2){

  analogWrite(u1F,speed1);
  analogWrite(u2F,speed2);
  analogWrite(u1B,0);
  analogWrite(u2B,0);

}

void turnleft(int speed1, int speed2){
  analogWrite(u1F,0);
  analogWrite(u2F,speed2);
  analogWrite(u1B,speed1);
  analogWrite(u2B,0);
}

void turnright(int speed1, int speed2){
  analogWrite(u1F,speed1);
  analogWrite(u2F,0);
  analogWrite(u1B,0);
  analogWrite(u2B,speed2);
}


void setup() {

  pinMode(u1F, OUTPUT);
  pinMode(u1B, OUTPUT);
  pinMode(u2F, OUTPUT);
  pinMode(u2B, OUTPUT);

  stop();

}

void loop() {

stop();

}

/*

// Archive 

loop_ctr++;

if (state) {
  pwm_ctr++;
} else {
  pwm_ctr--;
}

if (pwm_ctr >= max_speed) {
  state = false;
  pwm_ctr = max_speed; 
} 

if (pwm_ctr <= min_speed) {
  state = true;
  pwm_ctr = min_speed;
}

analogWrite(3,pwm_ctr);

delay(10);


if (loop_ctr >= 50) {
  Serial.println(pwm_ctr);
  loop_ctr = 0;
}

*/