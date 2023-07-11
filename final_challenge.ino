#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
const int leftTrigPin = 38;
const int leftEchoPin = 39;
const int rightTrigPin = 36;
const int rightEchoPin = 37;
const int servoPin = 11;
const int buttonPin = 26;
int in2 = 3;
int in1 = 4;

Servo servo;
boolean isStarted = false;
int orangeCount = 0;
int blueCount = 0;
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

void followBlock(int signature) {
  int servoAngle;
  if (signature == 4) {
    // Green block, calculate top right corner's x-coordinate
    int topRightX = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width;
    int targetX = 50; // Target x-coordinate for the green block
    int error = targetX - topRightX;
    if (error < -50) {
      servoAngle = 60;
    } else {
      servoAngle = map(error, -50, 50, 60, 120);
    }
  } else {
    // Red block, calculate top left corner's x-coordinate
    int targetX = 270; // Target x-coordinate for the red block
    int error = targetX - pixy.ccc.blocks[i].m_x;
    if (error > 50) {
      servoAngle = 120;
    } else {
      servoAngle = map(error, -50, 50, 60, 120);
    }
  }
  servo.write(servoAngle);
}

void loop() {
  if (!isStarted) {
    if (digitalRead(buttonPin) == LOW) {
      isStarted = true;
    }
    return;  // Skip the rest of the loop until the button is pressed
  }

  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;

  // Variables to track red/green blocks
  int redGreenCount = 0;
  int redGreenArea[2] = {0, 0};
  int redGreenCenter[2] = {0, 0};

  for (int i = 0; i < numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 2) {
      unsigned long currentOrangeBlockTime = millis();
      int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
      if (orangeDelay > blockDelay) {
        orangeCount++;
        previousOrangeBlockTime = currentOrangeBlockTime;
        Serial.print("orangeCount = ");
        Serial.println(orangeCount);
        Serial.print('\n');
      }
    } else if (pixy.ccc.blocks[i].m_signature == 5) {
      unsigned long currentBlueBlockTime = millis();
      int blueDelay = currentBlueBlockTime - previousBlueBlockTime;
      if (blueDelay > blockDelay) {
        blueCount++;
        previousBlueBlockTime = currentBlueBlockTime;
        Serial.print("blueCount = ");
        Serial.println(blueCount);
        Serial.print('\n');
      }
    } else if (pixy.ccc.blocks[i].m_signature == 1 || pixy.ccc.blocks[i].m_signature == 4) {
      // Red or green block detected
      if (redGreenCount < 2) {
        // Store area and center of the block
        redGreenArea[redGreenCount] = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        redGreenCenter[redGreenCount] = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
        redGreenCount++;
      }
    }
  }

  if (orangeCount >= 12 || blueCount >= 12) {
    if (stopTime == 0) {
      stopTime = millis() + 2000;
    } else {
      if (millis() >= stopTime) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        delay(60000);
      }
    }
  }

  if (redGreenCount == 1) {
    if (redGreenArea[0] > 6000) {
      followBlock(pixy.ccc.blocks[i].m_signature);
    } else {
      int servoAngle = map(redGreenCenter[0], 0, 320, 60, 120);
      servo.write(servoAngle);
    }
  } else if (redGreenCount == 2) {
    if (redGreenArea[0] > redGreenArea[1]) {
      if (redGreenArea[0] > 6000) {
        followBlock(pixy.ccc.blocks[i].m_signature);
      } else {
        int servoAngle = map(redGreenCenter[0], 0, 320, 60, 120);
        servo.write(servoAngle);
      }
    } else {
      if (redGreenArea[1] > 6000) {
        followBlock(pixy.ccc.blocks[i].m_signature);
      } else {
        int servoAngle = map(redGreenCenter[1], 0, 320, 60, 120);
        servo.write(servoAngle);
      }
    }
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    int leftDistance = getDistance(leftTrigPin, leftEchoPin);
    int rightDistance = getDistance(rightTrigPin, rightEchoPin);
    int difference = leftDistance - rightDistance;

    int mappedValue;
    if (difference < -45) {
      mappedValue = 125;
    } else if (difference > 45) {
      mappedValue = 55;
    } else if (difference > -10 && difference < 10) {
      mappedValue = 90;
    } else {
      mappedValue = map(difference, -45, 45, 125, 55);
    }

    servo.write(mappedValue);
  }

  delay(50);
}
