// src/main.h

#pragma once

// For standard integer types like uint32_t
#include <cstdint>

//================================================================================
// CAN Configuration
//================================================================================

//! The unique ID for this node on the CAN bus.
constexpr uint8_t NODE_ID = 1;

//! The CAN ID for receiving commands (e.g., 0x201 for NODE_ID 1).
constexpr uint16_t COMMAND_ID = 0x200 + NODE_ID;

//! The CAN ID for sending status updates (e.g., 0x181 for NODE_ID 1).
constexpr uint16_t STATUS_ID = 0x180 + NODE_ID;

//================================================================================
// Function Declarations
//================================================================================

/**
 * @brief Processes incoming CAN bus messages.
 * * This function should be called repeatedly in the main loop to check for
 * and handle new messages on the CAN bus. It is the main function for handling CAN.
 */
void handleCAN();