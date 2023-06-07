#include <Pixy2.h>

#define LED_PIN 2
#define SIGNATURE_ID 5

Pixy2 pixy;

void setup() {
  Serial.begin(9600);
  pixy.init();
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  static int signatureCount = 0;

  // Get blocks from Pixy
  int blockCount = pixy.ccc.getBlocks();

  // Check if any blocks are detected
  if (blockCount) {
    for (int i = 0; i < blockCount; i++) {
      // Check if the detected block matches the signature ID
      if (pixy.ccc.blocks[i].m_signature == SIGNATURE_ID) {
        signatureCount++;
      }
    }
  }

  // Check if signature count is equal to 12
  if (signatureCount == 12) {
    digitalWrite(LED_PIN, HIGH); // Turn on the LED
  } else {
    digitalWrite(LED_PIN, LOW); // Turn off the LED
  }

  Serial.print("Signature count: ");
  Serial.println(signatureCount);

  delay(500); // Add a delay between each iteration
}
