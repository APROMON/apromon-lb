#include <Pixy2.h>

Pixy2 pixy;

void setup() {
  Serial.begin(9600);
  pixy.init();
}

void loop() {
  int blocks;
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int blockX = pixy.ccc.blocks[i].m_x;
      int blockY = pixy.ccc.blocks[i].m_y;
      
      Serial.print("Detected object at (");
      Serial.print(blockX);
      Serial.print(", ");
      Serial.print(blockY);
      Serial.println(")");
    }
  }
  
  delay(100);
}