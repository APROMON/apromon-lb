#include <Pixy2.h>
#include <Servo.h>


Pixy2 pixy;
const int leftTrigPin = 38;
const int leftEchoPin = 39;
const int rightTrigPin = 44;
const int rightEchoPin = 45;
const int frontTrigPin = 31;
const int frontEchoPin = 30;
const int servoPin = 41;
const int buttonPin = 26;
const int in2 = 3;
const int in1 = 4;
const int enablePin = 5;

Servo servo;
boolean isStarted = false;
int orangeCount = 0;
int blueCount = 0;
int speed;
unsigned long previousOrangeBlockTime = 0;
unsigned long previousBlueBlockTime = 0;
const unsigned long blockDelay = 2000;  // Delay in milliseconds after detecting a block
unsigned long stopTime = 0;


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

void setMotorSpeed(int speed) {
  analogWrite(enablePin, speed);
}

void setup() {
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
//  servo.write(130);
  if (!isStarted) {
    if (digitalRead(buttonPin) == LOW) {
      isStarted = true;
    }
    return;  // Skip the rest of the loop until the button is pressed
  }

  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;
//  speed = 100;

  for (int i = 0; i < numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 2 && pixy.ccc.blocks[i].m_y > 100) {
      unsigned long currentOrangeBlockTime = millis();
      int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
      if (orangeDelay > blockDelay) {
        orangeCount++;
        previousOrangeBlockTime = currentOrangeBlockTime;
        Serial.print("orangeCount = ");
        Serial.println(orangeCount);
        Serial.print('\n');
      }
    } else if (pixy.ccc.blocks[i].m_signature == 5 && pixy.ccc.blocks[i].m_y > 100) {
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


 if (orangeCount >= 12 || blueCount >= 12) {
  if (stopTime == 0) {
    stopTime = millis() + 1750;
  } else {
    if (millis() >= stopTime) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      delay(60000);
    }
  }
 }

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //setMotorSpeed(speed);
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

  int mappedValue;
  if (difference < -30) {
    mappedValue = 170;
  } else if (difference > 30) {
    mappedValue = 80;
  } else if (difference > -5 && difference < 5) {
    mappedValue = 130;
  } else {
    mappedValue = map(difference, -40, 40, 160, 90);
  }

  int speed;
  if (mappedValue >= 115 && mappedValue <= 145) {
    setMotorSpeed(85);
  } else {
    setMotorSpeed(115);
  }
//  } else if (mappedValue <= 115) {
//    speed = map(mappedValue, 115, 130, 200, 225);
//    setMotorSpeed(speed);
//  } else {
//    setMotorSpeed(255);
//  }

  servo.write(mappedValue);
//  Serial.print("Steering angle:");
//  Serial.println(mappedValue);

delay(100);
}