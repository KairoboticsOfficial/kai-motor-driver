#include "can_control.h"

CANMotorControl::CANMotorControl(STM32_CAN* canBus) {
    can = canBus;
    
    // Initialize with safe values
    cmd.target_value = 0;
    cmd.control_mode = 1;  // Velocity control
    cmd.flags = 0;
    cmd.reserved = 0;
    
    // Initialize status
    status.position = 0;
    status.velocity = 0;
    status.current = 0;
    status.mode = 0;
    physical_target = 0.0;
}

void CANMotorControl::begin() {
    can->begin();
    can->setBaudRate(500000);
    Serial.println("CAN initialized");
    Serial.print("Node ID: "); Serial.println(NODE_ID);
    Serial.print("Command ID: 0x"); Serial.println(COMMAND_ID, HEX);
    Serial.print("Status ID: 0x"); Serial.println(STATUS_ID, HEX);
    Serial.println("All values use 0-255 units");
}

bool CANMotorControl::handleIncomingMessages() {
    if (can->read(rxMsg) == 1 && rxMsg.id == COMMAND_ID && rxMsg.len >= 4) {
        
        memcpy(&cmd, rxMsg.buf, 4); // Copies all 4 bytes into cmd (cmd.target_value, cmd.control_mode, cmd.flags, cmd.reserved)

        // Convert target from CAN units (0-255) to physical units
        physical_target = canToPhysical(cmd.target_value, cmd.control_mode);
        
        // Update state
        motor_enabled = (cmd.flags & 0x01) && !(cmd.flags & 0x02); // Check if bit0 = 1 (enabled) and bit1 = 0 (not emergency stop)
        current_mode = cmd.control_mode;
        
        // Update status
        status.mode = current_mode;

        // DEBUG
        Serial.print("Received: target=");
        Serial.print(cmd.target_value);
        Serial.print(" units, mode=");
        Serial.print(current_mode);
        Serial.print(", enabled=");
        Serial.println(motor_enabled);
        
        return true;
    }
    return false;
}

void CANMotorControl::sendStatus(float position, float angular_velocity, float current) {

    // Convert physical values to CAN units for transmission
    status.position = physicalToCan(position, 0);           // Position type
    status.velocity = physicalToCan(angular_velocity, 1);   // Velocity type
    status.current = physicalToCan(current, 2);             // Current type
    status.mode = current_mode;
    
    // Send
    txMsg.id = STATUS_ID;
    txMsg.len = 4;
    memcpy(txMsg.buf, &status, 4);

    can->write(txMsg);
    
    // if (can->write(txMsg) == 1) {
    //     Serial.print("Status sent: pos=");
    //     Serial.print(status.position);
    //     Serial.print(", vel=");
    //     Serial.print(status.velocity);
    //     Serial.print(", curr=");
    //     Serial.print(status.current);
    //     Serial.println();
    // } else {
    //     Serial.println("Failed to send status");
    // }
}

float CANMotorControl::canToPhysical(uint8_t can_value, uint8_t mode) {
    switch(mode) {
        case 0: // Position control (0-255 → 0 to 2π radians)
            return (float)can_value * MAX_POSITION / 255.0f;
            
        case 1: // Velocity control (0-255 → -25 to +25 rad/s, 127 = 0)
            return ((float)can_value / 255.0f) * (2 * MAX_ANGULAR_VELOCITY) - MAX_ANGULAR_VELOCITY;
            
        case 2: // Current/Torque control (0-255 → -10 to +10 A, 127 = 0)
            return ((float)can_value / 255.0f) * (2 * MAX_CURRENT) - MAX_CURRENT;
            
        default:
            return 0.0f;
    }
}

uint8_t CANMotorControl::physicalToCan(float physical_value, uint8_t type) {
    float can_float;
    
    switch(type) {
        case 0:
            {
                float normalized = fmod(physical_value + 2*PI, 2*PI); // Wrap to 0-2π
                can_float = normalized * 255.0f / MAX_POSITION;
            }
            break;
            
        case 1:
            can_float = ((physical_value + MAX_ANGULAR_VELOCITY) / (2 * MAX_ANGULAR_VELOCITY)) * 255.0f;
            break;
            
        case 2:
            can_float = ((physical_value + MAX_CURRENT) / (2 * MAX_CURRENT)) * 255.0f;
            break;
            
        default:
            can_float = 0.0f;
            break;
    }
    
    return (uint8_t)constrain(can_float, 0, 255);
}