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
int enablePin = 5;

Servo servo;
boolean isStarted = false;
int orangeCount = 0;
int blueCount = 0;
unsigned long previousOrangeBlockTime = 0;
unsigned long previousBlueBlockTime = 0;
const unsigned long blockDelay = 2000;  // Delay in milliseconds after detecting a block
unsigned long stopTime = 0;

int direction = 0; // Variable to store the direction (-1: Orange is higher, 1: Blue is higher)

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

  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;

  boolean foundOrange = false;
  boolean foundBlue = false;
  int orangeBottomY = 0;
  int blueBottomY = 0;

  for (int i = 0; i < numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 2) {
      if (pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height > 100) {
        unsigned long currentOrangeBlockTime = millis();
        int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
        if (orangeDelay > blockDelay) {
          orangeCount++;
          previousOrangeBlockTime = currentOrangeBlockTime;
          Serial.print("orangeCount = ");
          Serial.println(orangeCount);
          Serial.print('\n'); 
        }
        foundBlue = true;
        blueBottomY = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height;
      }
    } else if (pixy.ccc.blocks[i].m_signature == 5) {
      if (pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height > 100) {
        unsigned long currentBlueBlockTime = millis();
        int blueDelay = currentBlueBlockTime - previousBlueBlockTime;
        if (blueDelay > blockDelay) {
          blueCount++;
          previousBlueBlockTime = currentBlueBlockTime;
          Serial.print("blueCount = ");
          Serial.println(blueCount);
          Serial.print('\n');
        }
        foundBlue = true;
        blueBottomY = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height;
      }
    }
  }
  
  if (direction == 0) {
    if (foundOrange && foundBlue) {
      if (orangeBottomY > blueBottomY) {
        direction = -1;
        servo.write(125);
      } else {
        direction = 1;
        servo.write(55);
      }
    } else if (foundOrange) {
      direction = -1;
      servo.write(125);
    } else if (foundBlue){
      direction = 1;
      servo.write(55);
    }
    Serial.println(direction);
    direction = 0;
  }

  if (orangeCount == 12 || blueCount == 12) {
    if (stopTime == 0) {
      stopTime = millis() + 1500;
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
  analogWrite(enablePin, 255);
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
  if (leftDistance < 7) {
    mappedValue = 125;
  } else if (rightDistance < 7) {
    mappedValue = 55;
  } else if (difference < -45) {
    mappedValue = 125;
  } else if (difference > 45) {
    mappedValue = 55;
  } else if (difference > 0 && difference < 0) {
    mappedValue = 90;
  } else {
    mappedValue = map(difference, -45, 45, 125, 55);
  }

  servo.write(mappedValue);
}
