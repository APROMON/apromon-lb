// OPEN CHALLENGE


// include libraries and define components pins

#include <Pixy2.h>
#include <Servo.h>


Pixy2 pixy;
const int leftTrigPin = 38;
const int leftEchoPin = 39;
const int rightTrigPin = 36;
const int rightEchoPin = 37;
const int servoPin = 41;
const int buttonPin = 26;
const int in2 = 3;
const int in1 = 4;
const int enablePin = 5;

Servo servo;

// define necessary variables for future calculations

boolean isStarted = false;
int orangeCount = 0;
int blueCount = 0;
int speed;
unsigned long previousOrangeBlockTime = 0;
unsigned long previousBlueBlockTime = 0;
const unsigned long blockDelay = 1250;
unsigned long stopTime = 0;

// define getting the distance from an ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  return distance;
}

// define setting the DC motor's speed
void setMotorSpeed(int speed) {
  analogWrite(enablePin, speed);
}

void setup() {
  // defining each sensor's role (input/output)
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Use internal pull-up resistor for the button
  servo.attach(servoPin);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  Serial.begin(9600);  // Initialize serial communication
  pixy.init();
}

void loop() {
  if (!isStarted) {
    if (digitalRead(buttonPin) == LOW) {
      isStarted = true;
    }
    return;  // Skip the rest of the loop until the button is pressed
  }

// get information from the pixy cam
  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;
  speed = 100;

// establish some calculations and take more information of the detected blocks from the camera
  for (int i = 0; i < numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 2) {
      speed = 120;
      unsigned long currentOrangeBlockTime = millis();
      int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
      if (orangeDelay > blockDelay) {
        orangeCount++;
        previousOrangeBlockTime = currentOrangeBlockTime;
        Serial.print("orangeCount = ");
        Serial.println(orangeCount);
        Serial.print('\n');
      }
    } else if (pixy.ccc.blocks[i].m_signature == 6) {
      speed = 120;
      unsigned long currentBlueBlockTime = millis();
      int blueDelay = currentBlueBlockTime - previousBlueBlockTime;
      if (blueDelay > blockDelay) {
        blueCount++;
        previousBlueBlockTime = currentBlueBlockTime;
        Serial.print("blueCount = ");
        Serial.println(blueCount);
        Serial.print('\n');
      }
    }
  }

// stop the car/program once the camera has detected 12 orange or blue lines (each lap has 4 lines and the robot has to accomplish 3 laps, therefore 4x3 = 12 lines need to be detected)
 if (orangeCount >= 12 || blueCount >= 12) {
  if (stopTime == 0) {
    stopTime = millis() + 1000;
  } else {
    if (millis() >= stopTime) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      delay(60000);
    }
  }
 }

// Set motor direction and read sensor distances
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  int leftDistance = getDistance(leftTrigPin, leftEchoPin);
  int rightDistance = getDistance(rightTrigPin, rightEchoPin);
  int difference = leftDistance - rightDistance;

  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right Distance: ");
  Serial.print(rightDistance);
  Serial.print(" cm, Difference: ");
  Serial.print(difference);
  Serial.println(" cm");
  Serial.print('\n');

// move the robot based on the difference between the two left and right ultrasonic sensors
  int mappedValue;
  if (difference < -45) {
    mappedValue = 170;
  } else if (difference > 45) {
    mappedValue = 90;
  } else if (difference > -4 && difference < 4) {
    mappedValue = 133;
  } else {
    mappedValue = map(difference, -45, 45, 170, 90);
  }

  setMotorSpeed(speed);

  servo.write(mappedValue);
}
