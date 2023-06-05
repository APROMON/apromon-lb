#include <Pixy2.h>
#include <Servo.h>
#include <SPI.h>

Pixy2 pixy;
int in2 = 3;
int in1 = 4;
const int trigPin = 12;
const int echoPin = 13;
int servoPin = 11;
int led = 2;
Servo Servo1;
unsigned long startTime = 0;
bool delayFlag = false;

void setup() {
  Serial.begin(9600);
  pixy.init();
  Servo1.attach(servoPin);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(led, OUTPUT);
}

void loop() {
  int blocks;
  pixy.ccc.getBlocks();
  Servo1.write(90);
 

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int blockX = pixy.ccc.blocks[i].m_x;
      int blockY = pixy.ccc.blocks[i].m_y;
      int blockWidth = pixy.ccc.blocks[i].m_width;
      int blockHeight = pixy.ccc.blocks[i].m_height;
      int area = blockWidth * blockHeight;
      uint16_t signature = pixy.ccc.blocks[i].m_signature;
      int steeringAngle = map(blockX, 0, 316, 65, 125);
      String signatureName = String(signature);
      Servo1.write(steeringAngle);

      Serial.print("Detected object (");
      Serial.print(signatureName);
      Serial.print(") at (");
      Serial.print(blockX);
      Serial.print(", ");
      Serial.print(blockY);
      Serial.print("), ");
      Serial.print("Area: ");
      Serial.println(area);

      if (area > 6000) {
  Serial.println("Area > 15000: LED should turn on");
  digitalWrite(led, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  Servo1.write(65);
  delay(1400);
  Servo1.write(125);
  delay(1200);
  Servo1.write(90);

} else {
  Serial.println("Area <= 15000: LED should turn off");
  digitalWrite(led, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

    } 
  }
}
