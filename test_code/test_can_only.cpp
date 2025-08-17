/*
  CAN Protocol Test Only
  Tests CAN communication without running any motors
  
  This code only tests:
  - CAN message reception
  - Message parsing
  - State updates
  - Debug output
  
  NO MOTOR CONTROL - Safe for testing!
*/

#include <Arduino.h>
#include "STM32_CAN.h"
#include "can_control.h"

// CAN setup only
STM32_CAN Can1(PB_8, PB_9);
CANMotorControl canControl(&Can1);

// Variables to store CAN state
float can_target = 0.0f;
bool can_enabled = false;
uint8_t can_mode = 0;
bool can_emergency = false;

// Timing for debug output
unsigned long last_print = 0;
unsigned long message_count = 0;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== CAN Protocol Test Only ===");
  Serial.println("NO MOTOR CONTROL - Safe for testing!");
  
  // Initialize CAN only
  canControl.begin();
  
  Serial.println("\n=== Waiting for CAN Commands ===");
  Serial.println("Send CAN messages to test protocol:");
  Serial.println("- Command ID: 0x201");
  Serial.println("- 4 bytes: [target_low, target_high, mode, flags]");
  Serial.println("\nExample commands:");
  Serial.println("- Enable 0.5A:  [0xF4, 0x01, 0x02, 0x01]");
  Serial.println("- Disable:      [0x00, 0x00, 0x02, 0x00]"); 
  Serial.println("- Emergency:    [0x00, 0x00, 0x02, 0x02]");
  
  Serial.println("\n--- CAN Message Monitor ---");
  Serial.println("Time(ms) | Msg# | Target | Mode | Enable | Emergency | Raw Bytes");
  Serial.println("--------------------------------------------------------------------");
}

void loop() {
  // Check for CAN messages and update state
  if (canControl.handleIncomingMessages()) {
    message_count++;
    
    // Read all the parsed values
    can_target = canControl.getTarget();
    can_enabled = canControl.isEnabled();
    can_mode = canControl.getMode();
    can_emergency = canControl.isEmergencyStop();
    
    // Print immediate message info
    Serial.print(millis());
    Serial.print(" | ");
    Serial.print(message_count);
    Serial.print(" | ");
    Serial.print(can_target, 3);
    Serial.print(" | ");
    Serial.print(can_mode);
    Serial.print(" | ");
    Serial.print(can_enabled ? "YES" : "NO");
    Serial.print(" | ");
    Serial.print(can_emergency ? "YES" : "NO");
    Serial.print(" | ");
    
    // Show raw bytes for debugging
    Serial.print("[");
    Serial.print("Target: "); Serial.print(canControl.getTarget(), 3);
    Serial.print(", 0x"); Serial.print(canControl.getControlMode(), HEX);
    Serial.print(", 0x"); Serial.print(canControl.getFlags(), HEX);
    Serial.print("]");
    
    // Status indicators
    if (can_emergency) {
      Serial.print(" 🛑 ESTOP");
    } else if (can_enabled) {
      Serial.print(" ✅ ACTIVE");
    } else {
      Serial.print(" ⏸️ DISABLED");
    }
    
    Serial.println();
  }
  
  // Print periodic status every 2 seconds
  if (millis() - last_print > 2000) {
    Serial.println("");
    Serial.print("📊 STATUS - Messages received: "); Serial.print(message_count);
    Serial.print(", Current target: "); Serial.print(can_target, 3);
    Serial.print("A, Mode: "); Serial.print(can_mode);
    Serial.print(" ("); 
    if (can_mode == 0) Serial.print("Position");
    else if (can_mode == 1) Serial.print("Velocity"); 
    else if (can_mode == 2) Serial.print("Torque");
    else Serial.print("Unknown");
    Serial.print("), State: ");
    if (can_emergency) Serial.println("EMERGENCY STOP");
    else if (can_enabled) Serial.println("ENABLED");
    else Serial.println("DISABLED");
    Serial.println("");
    
    last_print = millis();
  }
  
  // Small delay to prevent spam
  delay(10);
}