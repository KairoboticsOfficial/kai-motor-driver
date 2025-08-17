/*
 * Minimal CAN Receiver - STM32
 * 
 * Just receives CAN messages and prints them
*/

/*
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
*/

#include <Arduino.h>
#include "STM32_CAN.h"

STM32_CAN Can1(PB_8, PB_9);
CAN_message_t msg;

void setup() {
  Serial.begin(115200);
  Serial.println("Minimal CAN Receiver");
  
  Can1.begin();
  Can1.setBaudRate(500000);
  
  Serial.println("Ready");
}

void loop() {
  if (Can1.read(msg) == 1) {
    Serial.print("ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" Data: ");
    
    for (int i = 0; i < msg.len; i++) {
      if (msg.buf[i] < 0x10) Serial.print("0");
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // React to specific messages
    if (msg.id == 0x400 && msg.buf[0] == 1) {
      Serial.println("-> MOTOR ON");
    }
    if (msg.id == 0x400 && msg.buf[0] == 0) {
      Serial.println("-> MOTOR OFF");
    }
  }
}