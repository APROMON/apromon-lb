#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo servo;

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

const unsigned long blockDelay = 4000;  // Delay in milliseconds after detecting a block

int leftDistance, rightDistance, frontDistance;
int redCenter, greenCenter, topRightX, topLeftX, Center;
boolean isStarted = false;
boolean detectedBlock = false;
boolean centeredBlock = false;
boolean foundOrange = false;
boolean foundBlue = false;
int direction = 0;
int signature = 0;
int orangeCount = 0;
int blueCount = 0;
unsigned long previousOrangeBlockTime = 0;
unsigned long previousBlueBlockTime = 0;
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

void followBlock(int signature) {
  int servoAngle;
  setMotorSpeed(110);

  if (signature == 1) {
    if (topRightX <= 80) {
      servoAngle = 130;
    } else if (topRightX <= 140) {
      servoAngle = map(topRightX, 80, 140, 140, 170);
    } else {
      servoAngle = 170;
    }
  } else {
    if (topLeftX >= 240) {
      servoAngle = 130;
    } else if (topLeftX >= 170) {
      servoAngle = map(topLeftX, 170, 240, 80, 120);
    } else {
      servoAngle = 80;
    }
  }
  servo.write(servoAngle);
}

void followCenter(int signature) {
  int servoAngle;
  setMotorSpeed(95);

  if (signature == 1) {
    if (leftDistance < 8) {
      servoAngle = 170;
    } else if (rightDistance < 8) {
      servoAngle = 80;
    } else {
      servoAngle = map(redCenter, 0, 320, 80, 170);
    }
  } else {
    if (leftDistance < 8) {
      servoAngle = 170;
    } else if (rightDistance < 8) {
      servoAngle = 80;
    } else {
      servoAngle = map(greenCenter, 0, 320, 80, 170);
    }
  }
  servo.write(servoAngle);
}

void setup() {
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Use internal pull-up resistor for the button
  servo.attach(servoPin);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(enablePin, OUTPUT);
  analogWrite(enablePin, 0);
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

  // Set motor direction and read sensor distances
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  setMotorSpeed(95);
  leftDistance = getDistance(leftTrigPin, leftEchoPin);
  rightDistance = getDistance(rightTrigPin, rightEchoPin);
  frontDistance = getDistance(frontTrigPin, frontEchoPin);
  int difference = leftDistance - rightDistance;

  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right Distance: ");
  Serial.print(rightDistance);
  Serial.print(" cm, Difference: ");
  Serial.print(difference);
  Serial.print(" cm, Front Distance: ");
  Serial.print(frontDistance);
  Serial.print('\n');

  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;

  int redGreenCount = 0;
  int redArea = 0;
  int greenArea = 0;
  int lowSpeed = 0;
  int redHeight = 0;
  int greenHeight = 0;
//  int servoSteering;
  int servoAngle = 0;
//  int mappedValue = 0;



  for (int i = 0; i < numBlocks; i++) {
    int blockSignature = pixy.ccc.blocks[i].m_signature;
    int blockHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height / 2;
    
    if (blockSignature == 2 && blockHeight > 50) {
      lowSpeed = 1;
      unsigned long currentOrangeBlockTime = millis();
      int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
      if (orangeDelay > blockDelay) {
        orangeCount++;
        previousOrangeBlockTime = currentOrangeBlockTime;
      }
    } else if (blockSignature == 5 && blockHeight > 50) {
      lowSpeed = 1;
      unsigned long currentBlueBlockTime = millis();
      int blueDelay = currentBlueBlockTime - previousBlueBlockTime;
      if (blueDelay > blockDelay) {
        blueCount++;
        previousBlueBlockTime = currentBlueBlockTime;
      }
    } else if (blockSignature == 1) {
      redCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width;
      topRightX = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
      redHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height / 2;
      redGreenCount++;
      if (redHeight > 120 && redCenter > 160) {
        servo.write(133);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        setMotorSpeed(150);
        delay(600);
      } 
    } else if (blockSignature == 4) {
      greenCenter = pixy.ccc.blocks[i].m_x;
      topLeftX = pixy.ccc.blocks[i].m_x - pixy.ccc.blocks[i].m_width / 2;
      greenHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height / 2;
      redGreenCount++;
      if (greenHeight > 120 && greenCenter < 160) {
        servo.write(133);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        setMotorSpeed(150);
        delay(600);
      } 
    }
  }

  if (orangeCount >= 12 || blueCount >= 12) {
    if (stopTime == 0) {
      stopTime = millis() + 5000;
    } else {
      if (millis() >= stopTime) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        setMotorSpeed(0);
        delay(60000);
      }
    }
  }

  if (redGreenCount > 0) {
    if (redHeight > greenHeight) {
      if (redHeight > 70) {
        followBlock(1);
        direction = 1;
        Serial.println("follow red");
      } else {
        followCenter(1);
        Serial.println("center red");
      }
    } else {
      if (greenHeight > 70) {
        followBlock(4);
        direction = 4;
        Serial.println("follow green");
      } else {
        followCenter(4);
        Serial.println("center green");
      }
    }
  } else if (direction == 1 && (frontDistance < 9 || rightDistance < 4)) {

    servo.write(170);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    setMotorSpeed(150);
    delay(700);

  } else if (direction == 4 && (frontDistance < 9 || leftDistance < 4)) {

    servo.write(80);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    setMotorSpeed(150);
    delay(700);

  } else {
 //    if (lowSpeed == 1){
 //      setMotorSpeed(100);
 //    } else {
 //      setMotorSpeed(80);
 //   }
    int mappedValue;
    if (leftDistance < 10 && rightDistance < 10) {
      servoAngle = 130;
    } else if (leftDistance < 5) {
      servoAngle = 170;
    } else if (rightDistance < 5) {
      servoAngle = 80;
    } else if (difference < -45) {
      servoAngle = 170;
    } else if (difference > 45) {
      servoAngle = 80;
    } else {
      servoAngle = map(difference, -45, 45, 170, 80);
    }

    servo.write(servoAngle);
    Serial.println("difference");
  
  }

  delay(100);

}

//  if (servoAngle == 0) {
//    servoSteering = mappedValue;
//  } else {
//    servoSteering = servoAngle;
//  }

//  int speed;
//  if (servoSteering > 130) {
//    speed = map(servoSteering, 130, 165, 60, 130);
//  } else {
//    speed = map(servoSteering, 85, 130, 130, 60);
//  }
//  setMotorSpeed(speed);
