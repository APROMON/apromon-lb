#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
const int leftTrigPin = 10;
const int leftEchoPin = 9;
const int rightTrigPin = 8;
const int rightEchoPin = 7;
const int servoPin = 11;
const int buttonPin = 26;
int in2 = 3;
int in1 = 4;
const int buzzerPin = 1;

Servo servo;
boolean isStarted = false;
int orangeCount = 0;
int blueCount = 0;
unsigned long previousOrangeBlockTime = 0;
unsigned long previousBlueBlockTime = 0;
const unsigned long blockDelay = 1000;  // Delay in milliseconds after detecting a block
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
  pinMode(buzzerPin, OUTPUT);        // Set buzzer pin as an output
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

  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;

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

  int mappedValue;
  if (difference < -30) {
    mappedValue = 130;
  } else if (difference > 30) {
    mappedValue = 50;
  } else if (difference > -0 && difference < 0) {
    mappedValue = 90;
  } else {
    mappedValue = map(difference, -60, 60, 130, 50);
  }

  servo.write(mappedValue);

  delay(100);
}
