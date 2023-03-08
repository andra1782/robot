/**
* Name: Andra Alazaroaie
*/

#define MAX 150

// sensor
int trigPin = PA9;
int echoPin = PB8;

// motors
int a1 = PB3;
int a2 = PA8;
int b1 = PB15;
int b2 = PB14;

unsigned long lastTrig = millis();
unsigned long trigDelay = 100;

volatile unsigned long startTime;
volatile unsigned long endTime;
volatile bool newDist = false;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  // the pin on which we want the interrupt, the ISR, interrupt when there is a CHANGE on the pin
  attachInterrupt(digitalPinToInterrupt(echoPin), echoPinInterrupt, CHANGE);
  goFront();
}

void loop() {
  // trigger only every trigDelay millis
  unsigned long currentTime = millis();
  if(currentTime - lastTrig > trigDelay) {
    lastTrig += trigDelay;
    trig();
  }

  // new distance detected by echo
  if(newDist) {
    newDist = false;
    double dist = getDistance();
    stopWhenClose(dist);
  }
}

// interrupt service routine
void echoPinInterrupt() {
  if(digitalRead(echoPin) == HIGH)
    startTime = micros();
  else {
    endTime = micros();
    newDist = true;
  }
}

// distance = speed * time formula, we use the speed of sound
double getDistance() {
  double time = endTime - startTime;
  return time / 58.0;
}

// trigger function for the ultrasound, transmit a short ultrasonic pulse for the echo to listen to
void trig() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

// function to stop the motors when we detect a distance smaller than 15
void stopWhenClose(double distance) {
  if(distance < 15.0) {
    stop();
    delay(10000);
  } else
    goFront();
}

void goFront() {
  analogWrite(a1, 0);
  analogWrite(a2, 185);
  analogWrite(b1, 0);
  analogWrite(b2, MAX);
}

void stop() {
  analogWrite(a1, 0);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 0);
}
