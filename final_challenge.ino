// OBSTACLE


// include libraries and define components pins

#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo servo;

const int leftTrigPin = 38;
const int leftEchoPin = 39;
const int rightTrigPin = 36;
const int rightEchoPin = 37;
const int frontTrigPin = 31;
const int frontEchoPin = 30;
const int servoPin = 41;
const int buttonPin = 26;
const int in2 = 3;
const int in1 = 4;
const int enablePin = 5;

// define necessary variables for future calculations
const unsigned long blockDelay = 4000;

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

// define the servo motor's input value by calculating the red/green block's position using the pixy v2.1 camera when the robot is close to it 
void followBlock(int signature) {
  int servoAngle = 130;
  int difference = leftDistance - rightDistance;
  setMotorSpeed(130);

  if (signature == 1) {
    if (difference < 6 && difference > -6) {
      servoAngle = 130;
    } else if (rightDistance < 10) {
      servoAngle = 80;
    } else if (leftDistance < 10) {
      servoAngle = 170;
    } else if (topRightX <= 60) {
      servoAngle = 130;
    } else if (topRightX <= 140) {
      servoAngle = map(topRightX, 60, 140, 140, 170);
    } else {
      servoAngle = 170;
    }
  } else {
    if (difference < 6 && difference > -6) {
      servoAngle = 130;
    } else if (rightDistance < 10) {
      servoAngle = 80;
    } else if (leftDistance < 10) {
      servoAngle = 170;
    } else if (topLeftX >= 260) {
      servoAngle = 130;
    } else if (topLeftX >= 170) {
      servoAngle = map(topLeftX, 170, 260, 80, 120);
    } else {
      servoAngle = 80;
    }
  }
  servo.write(servoAngle);
}

// define the servo motor's input value by calculating the red/green block's position using the pixy v2.1 camera when the robot is far from it 
void followCenter(int signature) {
  int servoAngle;

  if (signature == 1) {
    if (leftDistance < 8) {
      servoAngle = 170;
    } else if (rightDistance < 8) {
      servoAngle = 80;
    } else {
      servoAngle = map(redCenter, 0, 330, 80, 170);
    }
  } else {
    if (leftDistance < 8) {
      servoAngle = 170;
    } else if (rightDistance < 8) {
      servoAngle = 80;
    } else {
      servoAngle = map(greenCenter, 0, 330, 80, 170);
    }
  }
  servo.write(servoAngle);
}

void setup() {
  // defining each sensor's role (input/output)
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
  Serial.print(" cm, Front Distance: ");
  Serial.print(frontDistance);
  Serial.print('\n');

// get information for the pixy cam and define more values than need to be resetted to 0 when the loop repeats
  pixy.ccc.getBlocks();
  int numBlocks = pixy.ccc.numBlocks;

  int redGreenCount = 0;
  int redArea = 0;
  int greenArea = 0;
  int lowSpeed = 0;
  int redHeight = 0;
  int greenHeight = 0;
  int redBlockHeight = 0;
  int greenBlockHeight = 0;
  int servoAngle = 0;

// establish some calculations and take more information of the detected blocks from the camera
  for (int i = 0; i < numBlocks; i++) {
    int blockSignature = pixy.ccc.blocks[i].m_signature;
    int blockHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height;
    
    if (blockSignature == 2) {
      lowSpeed = 1;
      unsigned long currentOrangeBlockTime = millis();
      int orangeDelay = currentOrangeBlockTime - previousOrangeBlockTime;
      if (orangeDelay > blockDelay) {
        orangeCount++;
        previousOrangeBlockTime = currentOrangeBlockTime;
      }
    } else if (blockSignature == 6) {
      lowSpeed = 1;
      unsigned long currentBlueBlockTime = millis();
      int blueDelay = currentBlueBlockTime - previousBlueBlockTime;
      if (blueDelay > blockDelay) {
        blueCount++;
        previousBlueBlockTime = currentBlueBlockTime;
      }
    } else if (blockSignature == 1 && blockHeight > 50 ) {
      redArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      redCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
      topRightX = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width;
      redHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height;
      redGreenCount++;
      if (redBlockHeight > 80 && redCenter > 130 && redCenter < 190) {
        servo.write(100);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        setMotorSpeed(130);
        delay(800);
      } 
    } else if (blockSignature == 4 && blockHeight > 50) {
      greenArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      greenCenter = pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width / 2;
      topLeftX = pixy.ccc.blocks[i].m_x;
      greenHeight = pixy.ccc.blocks[i].m_y + pixy.ccc.blocks[i].m_height;
      redGreenCount++;
      if (greenBlockHeight > 80 && greenCenter > 130 && greenCenter < 190) {
        servo.write(100);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        setMotorSpeed(130);
        delay(800);
      } 
    }
  }

// stop the car/program once the camera has detected 12 orange or blue lines (each lap has 4 lines and the robot has to accomplish 3 laps, therefore 4x3 = 12 lines need to be detected)
  if (orangeCount >= 12 || blueCount >= 12) {
    if (stopTime == 0) {
      stopTime = millis() + 1500;
    } else {
      if (millis() >= stopTime) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        setMotorSpeed(0);
        delay(60000);
      }
    }
  }

// move the car based on the detedcted blocks positions
  if (redGreenCount > 0) {
    if (redHeight > greenHeight) {
      if (redHeight > 100) {
        followBlock(1);
        direction = 1;
      } else {
        followCenter(1);
      }
    } else {
      if (greenHeight > 100) {
        followBlock(4);
        direction = 4;
      } else {
        followCenter(4);
      }
    }
    // adjust the car if it has encountered a wall after passing a green block
  } else if (direction == 4 && frontDistance < 7 && frontDistance != 0) {
    detectedBlock = false;
    centeredBlock = false;

    while (!detectedBlock) {
      pixy.ccc.getBlocks();
      int numBlocks = pixy.ccc.numBlocks;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      servo.write(85);
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
        servo.write(100);
      } else {
        servo.write(130);
        centeredBlock = true;
        break;
      }
    }
    // adjust the car if it has encountered a wall after passing a red block
  } else if (direction == 1 && frontDistance <= 7 && frontDistance != 0) {
    detectedBlock = false;
    centeredBlock = false;

    while (!detectedBlock) {
      pixy.ccc.getBlocks();
      int numBlocks = pixy.ccc.numBlocks;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      servo.write(165);
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
        servo.write(145);
      } else {
        servo.write(130);
        centeredBlock = true;
        break;
      }
    }
    // move the robot based on the difference between the two left and right ultrasonic sensors
  } else {
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
  
  }

  setMotorSpeed(90);

  delay(100);

}
