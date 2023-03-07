/**
* Student name: Andra Alazaroaie
* Student number: 5518318
*
*/

#define MAX 150

enum dir {
  FRONT,
  RIGHT,
  LEFT,
  STOP
};

enum state {
  ON_LINE,
  BEFORE_BOX,
  LEFT_OF_BOX,
  AFTER_BOX,
  ON_LINE_AFTER
};

// motors
const int a1 = PB3;
const int a2 = PA8;
const int b1 = PB15;
const int b2 = PB14;

// infrared sensors
const int leftSensor = PA0;
const int rightSensor = PA1;

// ultrasound ranging sensors
int trigPinFront = PA9;
int echoPinFront = PB8;

int trigPinSide = PA10;
int echoPinSide = PB9;

int led = PC13;

// threshold for detecting the black line
const int threshold = 80;
dir currentDir = FRONT;
dir lastDir = FRONT;
// we start by following the line
state currentState = ON_LINE;

unsigned long lastTrig = millis();
unsigned long trigDelay = 100;

volatile unsigned long startTimeFront;
volatile unsigned long endTimeFront;
volatile bool newDistFront = false;

volatile unsigned long startTimeSide;
volatile unsigned long endTimeSide;
volatile bool newDistSide = false;

double prevDistFront = 50;
double prevDistSide = 50;

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinSide, OUTPUT);
  pinMode(echoPinSide, INPUT);

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  // the pin on which we want the interrupt, the ISR, interrupt when there is a CHANGE on the pin
  attachInterrupt(digitalPinToInterrupt(echoPinFront), ISRFront, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(echoPinSide), ISRSide, CHANGE); 

  goFront();
  delay(500);
}

void loop() {
  if(currentState == AFTER_BOX) // AFTER WE AVOIDED THE OBJECT WE LOOK FOR THE BLACK LINE AND STOP WHEN WE FOUND IT
    findLine();
  else if(currentState == ON_LINE_AFTER) // LINE FOLLOWER AFTER OBJECT AVOIDANCE
    followLine();
  else if(currentState == ON_LINE) { // LINE FOLLOWER BEFORE OBJECT AVOIDANCE
    followLine();
    // trigger the front ultrasound sensor only every trigDelay millis
    unsigned long currentTime = millis();
    if(currentTime - lastTrig > trigDelay) {
      lastTrig += trigDelay;
      trigFront();
    }
    // new distance detected by echo of the front ultrasound sensor
    if(newDistFront) {
      newDistFront = false;
      double dist = getDistanceFront();
      stopWhenClose(dist);
    }
  } else { // GO AROUND OBJECT
    goFront();
    // trigger the side ultrasound sensor only every trigDelay millis
    unsigned long currentTime = millis();
    if(currentTime - lastTrig > trigDelay) {
      lastTrig += trigDelay;
      trigSide();
    }
    // new distance detected by echo of the side ultrasound sensor
    if(newDistSide) {
      newDistSide = false;
      double dist = getDistanceSide();
      stopWhenFinishedSide(dist);
    }
  }
}

// function to stop the motors when we detect a distance smaller than 15 cm
void stopWhenClose(double distance) {
  if(distance < 15.0) {
    stop();
    changeState();
  }
}

// function to stop the motors when we detect a distance greater than 25 cm
void stopWhenFinishedSide(double distance) {
  if(distance > 25.0) {
    stop();
    delay(1000);
    changeState();
  }
}

// we have a FSM with 5 states, each coming right after the previous one
// the states represent our position relative to the object that we want to avoid
// according to the state we are in, there are certain actions to take before changing the state
// these are taking care of in this function
void changeState() {
  switch(currentState) {
    // once the object is detected, we turn left 90 degrees and go a tiny distance in front, then change the state
    case ON_LINE: 
      goLeft();
      delay(500);    
      prevDistSide = 0;
      lastTrig = millis();
      goFront();
      delay(500);
      currentState = BEFORE_BOX; break;
    // once we avoided the object from the front side and the side sensor doesn't detect anything anymore, 
    // we turn right 90 degrees, go a bit in front, then change the state
    case BEFORE_BOX: 
      goRight(); 
      delay(450); 
      goFront();
      delay(1500);
      prevDistSide = 0;
      currentState = LEFT_OF_BOX; break;
    // once we avoided the object from the side and the side sensor doesn't detect anything anymore,
    // we turn right 90 degrees, go a bit in front, then change the state
    case LEFT_OF_BOX:  
      goRight();
      delay(450);
      currentDir = FRONT;
      lastDir = STOP;      
      goFront();
      currentState = AFTER_BOX; break;
    // once we avoided the object from the back side and we detected the black line, we turn left 90 degrees,
    // then change state to the last one, which only continues following the line
    case AFTER_BOX: 
      digitalWrite(led, LOW);    
      goLeft();
      delay(550);
      stop();
      delay(1000);  
      currentDir = FRONT;
      lastDir = LEFT;      
      goFront();
      currentState = ON_LINE_AFTER; break;    
  }
}

// gets the distance detected by the front ultrasound sensor on an echo pin change
// to avoid jitter, we keep the previously detected distance and use it in the cases of a very large distance being detected,
// but also when calculating the new distance in a weighted mean, to avoid spikes in the distance calculation
double getDistanceFront() {
  double time = endTimeFront - startTimeFront;
  double dist = time / 58.8;
  if(dist > 600)
   dist = prevDistFront;
  dist = 0.6 * dist + 0.4 * prevDistFront;
  prevDistFront = dist;
  return dist;
}

// gets the distance detected by the side ultrasound sensor on an echo pin change
// to avoid jitter, we keep the previously detected distance and use it in the cases of a very large distance being detected,
// but also when calculating the new distance in a weighted mean, to avoid spikes in the distance calculation
double getDistanceSide() {
  double time = endTimeSide - startTimeSide;
  double dist = time / 58.8;
  if(dist > 600)
    dist = prevDistSide;
  dist = 0.6 * dist + 0.4 * prevDistSide;
  prevDistSide = dist;
  return dist;
}

// function for searching for the line and stopping once detecting it
void findLine() {
  // read the infrared sensors
  int leftSensorVal = analogRead(leftSensor);
  int rightSensorVal = analogRead(rightSensor);

  // we stop and change the state to the line following state once we found the line, otherwise we keep going front
  if (leftSensorVal > threshold || rightSensorVal > threshold){
    currentDir = STOP;
    stop();
    changeState();
  }else 
    currentDir = FRONT;
}

// line follower function
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
  changeDirection();
}

// we change the direction according to what we found, only if it is different from the previous one
inline void changeDirection() {
  if(currentDir != lastDir) {
    switch(currentDir) {
      case FRONT: goFront(); break;
      case RIGHT: goRight(); break;
      case LEFT: goLeft(); break;
      case STOP: stop(); break;
      default: stop(); break;
    }
  }
  lastDir = currentDir;
}

// interrupt service routine when a change is detected on the echo of the front ultrasound sensor
void ISRFront() {
  if(digitalRead(echoPinFront) == HIGH)
    startTimeFront = micros();
  else {
    endTimeFront = micros();
    newDistFront = true;
  }
}

// interrupt service routine when a change is detected on the echo of the side ultrasound sensor
void ISRSide() {
  if(digitalRead(echoPinSide) == HIGH)
    startTimeSide = micros();
  else {
    endTimeSide = micros();
    newDistSide = true;
  }
}

// trigger function for the front ultrasound sensor, transmit a short ultrasonic pulse for the echo to listen to
void trigFront() {
  digitalWrite(trigPinFront, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
}

// trigger function for the side ultrasound sensor, transmit a short ultrasonic pulse for the echo to listen to
void trigSide() {
  digitalWrite(trigPinSide, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSide, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSide, LOW);
}

// different duty cycles since one motor is more powerful
void goFront() {
  analogWrite(a1, 0);
  analogWrite(a2, 205);
  analogWrite(b1, 0);
  analogWrite(b2, 170);
}

void stop() {
  analogWrite(a1, 0);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 0);
  delay(500);
}

// turn the right motor front faster and the left motor slower in the opposite direction
void goRight() {
  analogWrite(a1, 170);
  analogWrite(a2, 0);
  analogWrite(b1, 0);
  analogWrite(b2, 190);
}

// turn the left motor front faster and the right motor slower in the opposite direction
void goLeft() {
  analogWrite(a1, 0);
  analogWrite(a2, 190);
  analogWrite(b1, 170);
  analogWrite(b2, 0);
}