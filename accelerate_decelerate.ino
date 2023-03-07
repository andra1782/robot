/**
* Student name: Andra Alazaroaie
* Student number: 5518318
*
*/
int a1 = PB3;
int a2 = PA8;
int b1 = PB15;
int b2 = PB14;

void setup() {
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
}

void loop() {
  // a bit above 50% duty cycle
  goFront(140);
  delay(3500);
  // accelerate
  for(int i = 140; i <= 255; i++) {
    goFront(i);
    delay(20);
  }
  delay(3000);
  // decelerate
  for(int i = 255; i >= 0; i--){
    goFront(i);
    delay(20);
  }
  // fully stopped
  delay(10000);
}

void goFront(int dc) {
  analogWrite(a1, 0);
  analogWrite(a2, dc);
  analogWrite(b1, 0);
  analogWrite(b2, dc);
}