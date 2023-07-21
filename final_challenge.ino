#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo servo;

const int leftTrigPin = 38;
const int leftEchoPin = 39;
const int rightTrigPin = 36;
const int rightEchoPin = 37;
const int frontTrigPin = 34;
const int frontEchoPin = 35;
const int servoPin = 11;
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
  int servoAngle = 90;

  if (signature == 1) {
    if (topRightX <= 20) {
      servoAngle = 90;
    } else if (topRightX <= 140) {
      servoAngle = map(topRightX, 20, 140, 100, 125);
    } else {
      servoAngle = 125;
    }
  } else {
    if (topLeftX >= 300) {
      servoAngle = 90;
    } else if (topLeftX >= 180) {
      servoAngle = map(topLeftX, 180, 300, 55, 80);
    } else {
      servoAngle = 55;
    }
  }
  servo.write(servoAngle);
}

void followCenter(int signature) {
  int servoAngle;

  if (signature == 1) {
    if (leftDistance < 8) {
      servoAngle = 125;
    } else if (rightDistance < 8) {
      servoAngle = 55;
    } else {
      servoAngle = map(redCenter, 0, 330, 55, 125);
    }
  } else {
    if (leftDistance < 8) {
      servoAngle = 125;
    } else if (rightDistance < 8) {
      servoAngle = 55;
    } else {
      servoAngle = map(greenCenter, 0, 330, 55, 125);
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
  Serial.print(" cm, Front Distance; ");
  Serial.print(frontDistance);
  Serial.print('\n');

  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;

  int redGreenCount = 0;
  int redArea = 0;
  int greenArea = 0;
  int lowSpeed = 0;


  for (int i = 0; i < numBlocks; i++) {
    int blockSignature = pixy.ccc.blocks[i].m_signature;
    int blockHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height;
    
    if (blockSignature == 2 && blockHeight > 100) {
      lowSpeed = 1;
      unsigned long currentOrangeBlockTime = millis();
      int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
      if (orangeDelay > blockDelay) {
        orangeCount++;
        previousOrangeBlockTime = currentOrangeBlockTime;
      }
    } else if (blockSignature == 6 && blockHeight > 100) {
      lowSpeed = 1;
      unsigned long currentBlueBlockTime = millis();
      int blueDelay = currentBlueBlockTime - previousBlueBlockTime;
      if (blueDelay > blockDelay) {
        blueCount++;
        previousBlueBlockTime = currentBlueBlockTime;
      }
    } else if (blockSignature == 1 && blockHeight > 100) {
      redArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      redCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
      topRightX = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width;
      redGreenCount++;
    } else if (blockSignature == 4 && blockHeight > 100) {
      greenArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      greenCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
      topLeftX = pixy.ccc.blocks[i].m_x;
      redGreenCount++;
    }
  }

  if (orangeCount >= 12 || blueCount >= 12) {
    if (stopTime == 0) {
      stopTime = millis() + 2500;
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
    setMotorSpeed(100);
    if (redArea > greenArea) {
      if (redArea > 2000) {
        setMotorSpeed(150);
        followBlock(1);
        direction = 1;
      } else {
        followCenter(1);
      }
    } else {
      if (greenArea > 2000) {
        setMotorSpeed(150);
        followBlock(4);
        direction = 4;
      } else {
        followCenter(4);
      }
    }
  } else if (direction == 4 && frontDistance < 15) {
    detectedBlock = false;
    centeredBlock = false;

    while (!detectedBlock) {
      pixy.ccc.getBlocks();
      int numBlocks = pixy.ccc.numBlocks;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      setMotorSpeed(100);
      servo.write(65);
      for (int i = 0; i < numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 4) {
          detectedBlock = true;
          break;
        }
      }
    }
    while (!centeredBlock) {
      pixy.ccc.getBlocks();
      int numBlocks = pixy.ccc.numBlocks;
      for (int i = 0; i < numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 4) {
          greenCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
        }
      }
      if (greenCenter > 200) {
        servo.write(75);
      } else {
        servo.write(90);
        centeredBlock = true;
        break;
      }
    }
  } else if (direction == 1 && frontDistance < 15) {
    detectedBlock = false;
    centeredBlock = false;

    while (!detectedBlock) {
      pixy.ccc.getBlocks();
      int numBlocks = pixy.ccc.numBlocks;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      setMotorSpeed(100);
      servo.write(115);
      for (int i = 0; i < numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 1) {
          detectedBlock = true;
          break;
        }
      }
    }
    while (!centeredBlock) {
      pixy.ccc.getBlocks();
      int numBlocks = pixy.ccc.numBlocks;
      for (int i = 0; i < numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 1) {
          redCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
        }
      }
      if (redCenter > 120) {
        servo.write(105);
      } else {
        servo.write(90);
        centeredBlock = true;
        break;
      }
    }
  } else {
    if (lowSpeed == 1){
      setMotorSpeed(70);
    } else {
      setMotorSpeed(100);
    }
    int mappedValue;
    if (leftDistance < 10 && rightDistance < 10) {
      mappedValue = 90;
    } else if (leftDistance < 5) {
      mappedValue = 125;
    } else if (rightDistance < 5) {
      mappedValue = 55;
    } else if (difference < -45) {
      mappedValue = 125;
    } else if (difference > 45) {
      mappedValue = 55;
    } else {
      mappedValue = map(difference, -45, 45, 125, 55);
    }

    servo.write(mappedValue);
  }
}
