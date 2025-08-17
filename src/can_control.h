// can_control.h
#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

#include "STM32_CAN.h"
#include <Arduino.h>

// CAN Configuration
#define NODE_ID 0x01 // If motor 2, use 0x02
#define COMMAND_ID (0x200 + NODE_ID)  // 0x201
#define STATUS_ID (0x300 + NODE_ID)   // 0x301

// Max values for CAN units
#define MAX_POSITION (2*PI)
#define MAX_ANGULAR_VELOCITY 20
#define MAX_CURRENT 40

/**
 * @brief Command structure for CAN motor control (4 bytes total)
 * Uses 0-255 units over CAN, converted to physical units internally
 */
struct MotorCommand {
  uint8_t target_value;    // 1 byte: 0-255 units (converted to physical)
  uint8_t control_mode;    // 1 byte: 0=position, 1=velocity, 2=current  
  uint8_t flags;           // 1 byte: bit0=enable, bit1=stop
  uint8_t reserved;        // 1 byte: future use
};

/**
 * @brief Status structure for CAN motor feedback (4 bytes total)
 * Uses 0-255 units over CAN, converted from physical units internally
 */
struct MotorStatus {
  uint8_t position;         // 1 byte: 0-255 position units
  uint8_t velocity;         // 1 byte: 0-255 velocity units
  uint8_t current;          // 1 byte: 0-255 current units
  uint8_t mode;             // 1 byte: mode(0-3) + status flags(4-7)
};

/**
 * @class CANMotorControl
 * @brief Handles CAN bus communication for motor control
 * Converts between 0-255 CAN units and physical units automatically
 */
class CANMotorControl {
    
private:
    STM32_CAN* can;
    CAN_message_t rxMsg, txMsg;
    MotorCommand cmd;
    MotorStatus status;
    
    // Physical target value (converted from CAN units)
    float physical_target;
    
    /**
     * @brief Convert CAN units (0-255) to physical units
     * @param can_value CAN unit value (0-255)
     * @param mode Control mode (0=position, 1=velocity, 2=current)
     * @return Physical value (radians, rad/s, or amps)
     */
    float canToPhysical(uint8_t can_value, uint8_t mode);

public:
    bool motor_enabled = false;
    uint8_t current_mode = 1;  // Default to velocity control
    unsigned long last_status_send = 0;
    unsigned long status_send_interval = 20; // Send status every 20ms (50Hz)

    /**
     * @brief Constructor - initializes CAN motor control interface
     * @param canBus Pointer to initialized STM32_CAN object
     */
    CANMotorControl(STM32_CAN* canBus);
        
    /**
     * @brief Initialize CAN bus communication
     */
    void begin();
        
    /**
     * @brief Read and parse incoming CAN command messages
     * Automatically converts CAN units to physical units
     * @return true if a valid command was received and state updated
     * @return false if no valid command message was available
     */
    bool handleIncomingMessages();

    /**
     * @brief Send motor status via CAN bus (converts physical to CAN units)
     * @param position_rad Current motor position (radians)
     * @param velocity_rad_s Current motor velocity (rad/s)  
     * @param current_a Current motor current (amps)
     */
    void sendStatus(float position_rad, float velocity_rad_s, float current_a);

    /**
     * @brief Convert physical units to CAN units (0-255)
     * @param physical_value Physical value
     * @param type Value type (0=position, 1=velocity, 2=current)
     * @return CAN unit value (0-255)
     */
    uint8_t physicalToCan(float physical_value, uint8_t type);

    // Getter functions - return physical units directly!
    float getTargetPosition() { return physical_target; }  // Returns radians
    float getTargetVelocity() { return physical_target; }  // Returns rad/s
    float getTargetCurrent() { return physical_target; }   // Returns amps
    float getTargetValue() { return physical_target; }     // Generic getter - returns physical units

    bool isEnabled() { return motor_enabled; }
    bool isEmergencyStop() { return cmd.flags & 0x02; }
    uint8_t getControlMode() { return cmd.control_mode; }
    uint8_t getFlags() { return cmd.flags; }
    
};

#endif