#include <Pixy2.h>

Pixy2 pixy;
const int buzzerPin = 7;
const int ledPin1 = 5;
const int ledPin5 = 6;

void setup() {
  Serial.begin(9600);
  pixy.init();
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin5, OUTPUT);
}

void loop() {
  int blocks;
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int blockX = pixy.ccc.blocks[i].m_x;
      int blockY = pixy.ccc.blocks[i].m_y;
      int blockWidth = pixy.ccc.blocks[i].m_width;
      int blockHeight = pixy.ccc.blocks[i].m_height;
      int area = blockWidth * blockHeight;
      uint16_t signature = pixy.ccc.blocks[i].m_signature;
      String signatureName = String(signature);
      
      Serial.print("Detected object (");
      Serial.print(signatureName);
      Serial.print(", ");
      Serial.print(signature);
      Serial.print(") at (");
      Serial.print(blockX);
      Serial.print(", ");
      Serial.print(blockY);
      Serial.print("), ");
      Serial.print("Area: ");
      Serial.println(area);

      if (area > 15000) {
        // Activate the buzzer on pin 13
        digitalWrite(buzzerPin,  HIGH);
      } else {
        // Deactivate the buzzer
        digitalWrite(buzzerPin,  LOW);
      }

      if (signature == 1) {
        // Turn on the LED on pin 11
        digitalWrite(ledPin1, HIGH);
      } else {
        // Turn off the LED on pin 11
        digitalWrite(ledPin1, LOW);
      }

      if (signature == 5) {
        // Turn on the LED on pin 12
        digitalWrite(ledPin5, HIGH);
      } else {
        // Turn off the LED on pin 12
        digitalWrite(ledPin5, LOW);
      }
    }
  } else {
    // Deactivate the buzzer and turn off the LEDs if no blocks detected
    noTone(buzzerPin);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin5, LOW);
  }
  
  delay(100);
}
