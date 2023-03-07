/**
* Student name: Andra Alazaroaie
* Student number: 5518318
*
*/
#define PI 3.1415926535897932384626433832795
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
const int threshold = 150;
dir currentDir = FRONT;

// speed encoders
const int leftEncoder = PB12;
const int rightEncoder = PB13;

// properties of the speed encoders
const double radius = 3.5;
const double steps = 20.0;

// counters for the steps of the speed encoders
int counterLeft = 0;
int counterRight = 0;
// last states of the speed encoders
int lastLeft = LOW;
int lastRight = LOW;
// the distance that we want to travel
const double fixedDistanceInCm = 142.0;

void setup() {
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  pinMode(leftEncoder, INPUT_PULLUP);
  pinMode(rightEncoder, INPUT_PULLUP);

  goFront();
}

void loop() {
  followLine();
}

// read the left encoder, detect any change of state (low to high or high to low)
void readLeftEncoder() {
  int read = digitalRead(leftEncoder);
  if(read != lastLeft)
    counterLeft++; // Clockwise
  lastLeft = read;
}

// read the right encoder, detect any change of state (low to high or high to low)
void readRightEncoder() {
  int read = digitalRead(rightEncoder);
  if(read != lastRight)
    counterRight++; // Clockwise
  lastRight = read;
}

void followLine() {
  // read the infrared sensors
  int leftSensorVal = analogRead(leftSensor);
  int rightSensorVal = analogRead(rightSensor);

  // we set the current direction to the one of the sensor that detects a value greater than the threshold
  if (leftSensorVal <= threshold && rightSensorVal <= threshold)
    currentDir = FRONT;
  else if (leftSensorVal <= threshold && rightSensorVal > threshold)
    currentDir = RIGHT;
  else if (leftSensorVal > threshold && rightSensorVal <= threshold)
    currentDir = LEFT;
  else 
    currentDir = FRONT;

  // we change the direction according to what we found, for turns we stop for a tiny bit just so that we can go slower and detect the sharpest curves
  switch(currentDir) {
    case FRONT: goFront(); break;
    case RIGHT: goRight(); break;
    case LEFT: goLeft(); break;
    default: stop(); break;
  }
}

// different duty cycles since one motor is more powerful
void goFront() {
  analogWrite(a1, 0);
  analogWrite(a2, 185);
  analogWrite(b1, 0);
  analogWrite(b2, MAX);

  // read both speed encoders
  readLeftEncoder();
  readRightEncoder();

  //calculate the distance travelled according to the speed encoders' readings and properties
  double avg = (counterLeft + counterRight) / 2;
  double distance = ((2 * PI * radius) / steps) * avg / 2;

  // stop when the distance calculated reached the desired distance
  if(distance >= fixedDistanceInCm){
    stop();
    delay(10000);
  } 
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
  analogWrite(b2, 170);
}

// turn the left motor front faster and the right motor slower in the opposite direction
void goLeft() {
  analogWrite(a1, 0);
  analogWrite(a2, 170);
  analogWrite(b1, 150);
  analogWrite(b2, 0);
}