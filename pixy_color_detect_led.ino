#include <SPI.h>
#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo Servo1;

int red = 10;
int blue = 7;
int servo = 9;
int buzz = 8; 

void setup() {
  Serial.begin(9600);
  pixy.init();
  Servo1.attach(servo);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(servo, OUTPUT);
}

void loop() {
  int blocks;
  char buf[32];

  blocks = pixy.ccc.getBlocks();

digitalWrite(blue, LOW); 
   digitalWrite(red, LOW);
   digitalWrite(buzz, LOW); 
  if (blocks) {
    for (int i = 0; i < blocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        digitalWrite(red, HIGH); 
        Serial.println ("red");
        Servo1.write(0);
       // Set color to red for signature 1
        break;  // Exit the loop after detecting signature 1
       
      } else if (pixy.ccc.blocks[i].m_signature == 5) {
        digitalWrite(blue, HIGH);
        Serial.println ("blue");
        Servo1.write(150); // Set color to blue for signature 3
        break;  // Exit the loop after detecting signature 3
      }
      // Add more if-else conditions for other signatures if needed
    }
  } else {
    Servo1.write(90);
   digitalWrite(blue, LOW); 
   digitalWrite(red, LOW);
   digitalWrite(buzz, LOW);
   Serial.println ("no detection"); // No signature detected, turn off the RGB module
  }
}

