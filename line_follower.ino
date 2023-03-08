/**
* Student name: Andra Alazaroaie
*/

#define MAX 150

enum dir {
  FRONT,
  RIGHT,
  LEFT,
  STOP
};

// motors
const int a1 = PB3;
const int a2 = PA8;
const int b1 = PB15;
const int b2 = PB14;

// infrared sensors
const int leftSensor = PA0;
const int rightSensor = PA1;

// threshold for detecting the black line
const int threshold = 80;
dir current = STOP;

void setup() {
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  pinMode (leftSensor, INPUT);
  pinMode (rightSensor, INPUT);

  goFront();
}

void loop() {
  // read the infrared sensors
  int leftSensorVal = analogRead(leftSensor);
  int rightSensorVal = analogRead(rightSensor);

  // we set the current direction to the one of the sensor that detects a value greater than the threshold
  if (leftSensorVal <= threshold && rightSensorVal <= threshold)
    current = FRONT;
  else if (leftSensorVal <= threshold && rightSensorVal > threshold)
    current = RIGHT;
  else if (leftSensorVal > threshold && rightSensorVal <= threshold)
    current = LEFT;
  else 
    current = FRONT;

  // we change the direction according to what we found, for turns we stop for a tiny bit just so that we can go slower and detect the sharpest curves
  switch(current) {
    case FRONT: goFront(); break;
    case RIGHT: stop(); delay(10); goRight(); break;
    case LEFT: stop(); delay(10); goLeft(); break;
    default: stop(); break;
  }

  delay(30);
  // we also stop here for a bit to go slow and detect small and sharp curves
  stop();
  delay(5); 
}

void goFront() {
  analogWrite(a1, 0);
  analogWrite(a2, MAX);
  analogWrite(b1, 0);
  analogWrite(b2, MAX);
}

void stop() {
  analogWrite(a1, 0);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 0);
}

// turn the right motor front faster and the left motor slower in the opposite direction
void goRight() {
  analogWrite(a1, 150);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 175);
}

// turn the left motor front faster and the right motor slower in the opposite direction
void goLeft() {
  analogWrite(a1, 0);
  analogWrite(a2, 175);
  analogWrite(b1, 150);
  analogWrite(b2, 0);
}
