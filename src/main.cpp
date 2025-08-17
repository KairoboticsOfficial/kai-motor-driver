 /*
  Torque Control Test with Current Monitoring
  SimpleFOC MKS DRIVE MINI + AS5047P + Motor
  With DRV8301 Gain Control
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include "STM32_CAN.h"
#include "can_control.h"

// DRV8301 Register Addresses
#define STATUS_REGISTER1_ADDR    0x00
#define STATUS_REGISTER2_ADDR    0x01
#define CONTROL_REGISTER1_ADDR   0x02
#define CONTROL_REGISTER2_ADDR   0x03

// Gain settings
#define GAIN_10V            0b00
#define GAIN_20V            0b01
#define GAIN_40V            0b10
#define GAIN_80V            0b11

// Motor parameters
#define PP 14

// Driver pins
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
#define EN_GATE PB12

// Current sensing pins
#define M0_IA _NC // Phase A not available
#define M0_IB PC0 // Phase B current sense
#define M0_IC PC1 // Phase C current sense

// Encoder pins
#define CS PA15
#define SPI_MISO PC11
#define SPI_MOSI PC12
#define SPI0SCK PC10

// DRV8301 chip select
#define DRV8301_CS PC13

// Serial communication
#define PIN_SERIAL1_RX PA3
#define PIN_SERIAL1_TX PA2
HardwareSerial Serial1(PIN_SERIAL1_RX, PIN_SERIAL1_TX);

// CAN
#define CAN_TIMEOUT_MS 100

// Motor, driver, and sensor instances
BLDCMotor motor = BLDCMotor(PP);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 80.0f, M0_IA, M0_IB, M0_IC);

// Encoder setup
MagneticSensorSPIConfig_s AS5047P_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 4000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13, 
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorSPI sensor(AS5047P_SPI, CS);
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

// Variables for monitoring
PhaseCurrent_s current;
float current_magnitude;
long timestamp = 0;
float target_current = 0.0f;
float motor_velocity = 0.0f;

// CAN setup
STM32_CAN Can1(PB_8, PB_9);
CANMotorControl canControl(&Can1);

// Timing for status updates
unsigned long last_status_time = 0;
unsigned long lastCANMessageTime = 0;
const unsigned long STATUS_INTERVAL = 50; // Send status every 50ms

#define DRV8301_SPI_SETTINGS SPISettings(250000, MSBFIRST, SPI_MODE1)

// DRV8301 Functions

/**
 * @brief Writes data to a register on the DRV8301 using a dedicated SPI transaction.
 */
void DRV8301_WriteRegister(uint8_t address, uint16_t data) {
    uint16_t frame = (address << 11) | (data & 0x7FF);
    
    SPI_2.beginTransaction(DRV8301_SPI_SETTINGS);
    digitalWrite(DRV8301_CS, LOW);
    delayMicroseconds(5);
    SPI_2.transfer16(frame);
    delayMicroseconds(5);
    digitalWrite(DRV8301_CS, HIGH);
    SPI_2.endTransaction();
}

/**
 * @brief Reads a register from the DRV8301 using a dedicated SPI transaction.
 */
uint16_t DRV8301_ReadRegister(uint8_t address) {
    uint16_t cmd = 0x8000 | (address << 11);
    uint16_t result;

    SPI_2.beginTransaction(DRV8301_SPI_SETTINGS);
    
    // First transaction: send the read command
    digitalWrite(DRV8301_CS, LOW);
    delayMicroseconds(5);
    SPI_2.transfer16(cmd);
    delayMicroseconds(5);
    digitalWrite(DRV8301_CS, HIGH);
    
    delayMicroseconds(20); // Wait for DRV to prepare data
    
    // Second transaction: send NOP to read the data
    digitalWrite(DRV8301_CS, LOW);
    delayMicroseconds(5);
    result = SPI_2.transfer16(0x0000);
    delayMicroseconds(5);
    digitalWrite(DRV8301_CS, HIGH);
    
    SPI_2.endTransaction();
    
    return result & 0x7FF;
}

/**
 * @brief Sets the gain of the DRV8301's current sense amplifiers.
 */
bool DRV8301_SetGain(uint8_t gain_setting) {
    uint16_t ctrl2 = DRV8301_ReadRegister(CONTROL_REGISTER2_ADDR);
    Serial.print("DRV8301 Control Reg 2 before: 0x");
    Serial.println(ctrl2, HEX);
    
    ctrl2 &= ~(0x3 << 2);
    ctrl2 |= (gain_setting << 2);
    
    Serial.print("Attempting to write 0x");
    Serial.print(ctrl2, HEX);
    Serial.println(" to Control Reg 2...");
    
    DRV8301_WriteRegister(CONTROL_REGISTER2_ADDR, ctrl2);
    delay(1);
    
    uint16_t new_ctrl2 = DRV8301_ReadRegister(CONTROL_REGISTER2_ADDR);
    Serial.print("DRV8301 Control Reg 2 after:  0x");
    Serial.println(new_ctrl2, HEX);
    
    uint8_t actual_gain = (new_ctrl2 >> 2) & 0x3;
    Serial.print("DRV8301 Gain set to: ");
    Serial.print(10 * (1 << actual_gain));
    Serial.println("V/V");
    
    return actual_gain == gain_setting;
}

/**56
 * @brief Initializes the DRV8301 driver, clears faults, and verifies communication.
 */
bool DRV8301_Init() {
    Serial.println("\n=== Initializing DRV8301 ===");
    
    Serial.println("Resetting DRV8301 logic...");
    DRV8301_WriteRegister(CONTROL_REGISTER1_ADDR, 0x0002);
    delay(10);
    DRV8301_WriteRegister(CONTROL_REGISTER1_ADDR, 0x0000);
    delay(10);

    DRV8301_ReadRegister(STATUS_REGISTER1_ADDR);
    delay(1);
    
    uint16_t deviceID = DRV8301_ReadRegister(STATUS_REGISTER2_ADDR) & 0xF;
    
    if (deviceID != 0x1) {
        Serial.print("✗ DRV8301 not detected! Read Device ID: 0x");
        Serial.println(deviceID, HEX);
        return false;
    }
    
    Serial.println("✓ DRV8301 detected!");
    
    uint16_t status1 = DRV8301_ReadRegister(STATUS_REGISTER1_ADDR);
    if (status1 & 0x400) {
        Serial.print("WARNING: DRV8301 fault detected after reset! Status: 0x");
        Serial.println(status1, HEX);
    }
    
    return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  SimpleFOCDebug::enable(&Serial);

  // Initialize all chip select pins as outputs and set HIGH (inactive).
  pinMode(DRV8301_CS, OUTPUT);
  digitalWrite(DRV8301_CS, HIGH);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, HIGH);
  delay(10);

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize SPI▬▬▬▬▬▬▬▬▬▬▬▬
  SPI_2.begin();
  delay(100);

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize DRV8301▬▬▬▬▬▬▬▬▬▬▬▬
  digitalWrite(CS, HIGH);
  if (!DRV8301_Init()) {
    Serial.println("Halting due to DRV8301 initialization failure.");
    while(1);
  }

  //▬▬▬▬▬▬▬▬▬▬▬▬Set DRV8301 Gain▬▬▬▬▬▬▬▬▬▬▬▬
  Serial.println("\nSetting DRV8301 operational gain to 40V/V...");
  if (!DRV8301_SetGain(GAIN_80V)) {
    Serial.println("WARNING: Failed to set DRV8301 gain! Check wiring/power.");
  }

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize CAN▬▬▬▬▬▬▬▬▬▬▬▬
  Serial.println("\nInitializing CAN...");
  canControl.begin();

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize encoder AS5047P▬▬▬▬▬▬▬▬▬▬▬▬
  Serial.println("\nInitializing encoder...");
  sensor.init(&SPI_2);
  motor.linkSensor(&sensor);
  Serial.println("✓ Encoder initialized");

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize driver▬▬▬▬▬▬▬▬▬▬▬▬
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 20.0f;
  driver.init();
  motor.linkDriver(&driver);
  Serial.println("✓ Driver initialized");

  currentSense.linkDriver(&driver);

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize motor▬▬▬▬▬▬▬▬▬▬▬▬
  motor.velocity_limit = 25.0f;
  motor.voltage_limit = 20.0f;
  
  // CHANGED: The motor's current_limit now acts as the TORQUE limit for the velocity controller.
  motor.current_limit = 10.0f; 
  
  motor.phase_resistance = 0.214f;
  motor.torque_controller = TorqueControlType::foc_current;
  
  // ====================================================================
  // >> CHANGED: We are now in VELOCITY control mode <<
  motor.controller = MotionControlType::velocity;
  // ====================================================================

  // These are your inner-loop torque gains. They are already well-tuned. DO NOT CHANGE.
  motor.PID_current_q.P = 0.7f;
  motor.PID_current_q.I = 50.0f;
  motor.PID_current_d.P = 0.7f;
  motor.PID_current_d.I = 50.0f;
  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  // ====================================================================
  // >> NEW: Add PID and LPF gains for the new VELOCITY loop <<
  // Start with low values and tune these just like you did for torque.
  motor.PID_velocity.P = 1.2f;    // Proportional gain
  motor.PID_velocity.I = 0.5f;    // Integral gain
  motor.PID_velocity.D = 0.0f;    // Derivative gain, leave at 0 for now
  motor.LPF_velocity.Tf = 0.03f;  // Low-pass filter for velocity measurement // Below 0.02 we get more noise. 0.025 can work too. Try 0.03
  // ====================================================================
  
  motor.init();

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize and Calibrate Current Sense▬▬▬▬▬▬▬▬▬▬▬▬
  if (currentSense.init()) {
    Serial.println("✓ Current sensing initialized and calibrated.");
    motor.linkCurrentSense(&currentSense);
  } else {
    Serial.println("✗ Current sensing failed! Halting.");
    while(1);
  }

  //▬▬▬▬▬▬▬▬▬▬▬▬Initialize FOC▬▬▬▬▬▬▬▬▬▬▬▬
  motor.initFOC();

  Serial.println("\n=== Starting Velocity Control Test ===");
}

void loop() {
  // This function executes the FOC algorithm. Always run it as fast as possible.
  motor.loopFOC();

  // Parse CAN messages
  if (canControl.handleIncomingMessages()){
    lastCANMessageTime = millis();
    
    uint8_t new_mode = canControl.getControlMode();
    
    Serial.print("CAN command received - Mode: ");
    Serial.print(new_mode);
    Serial.print(" | Target: ");
    Serial.print(canControl.getTargetValue());
    Serial.print(" | Enabled: ");
    Serial.print(canControl.isEnabled());
    Serial.print(" | E-Stop: ");
    Serial.println(canControl.isEmergencyStop());
    
    if (!canControl.isEmergencyStop()) {
      uint8_t current_mode;

      switch(motor.controller) {
        case MotionControlType::angle:
          current_mode = 0;
          break;

        case MotionControlType::velocity:
          current_mode = 1;
          break;

        case MotionControlType::torque:
          current_mode = 2;
          break;

        default:
          current_mode = 255; // Invalid
          break;
      }

      if (new_mode != current_mode){
        // Switch control modes based on CAN command, and only when it's different

        switch (new_mode) { // 0=position, 1=velocity, 2=torque 
          case 0:
            Serial.println("Position control");
            motor.controller = MotionControlType::angle;
            break;

          case 1:
            Serial.println("Velocity control");
            motor.controller = MotionControlType::velocity;
            break;

          case 2:
            Serial.println("Torque control");
            motor.controller = MotionControlType::torque;
            break;
        };
      };
    };
  };

  if (millis() - lastCANMessageTime > CAN_TIMEOUT_MS || canControl.isEmergencyStop()) {
    // Disable motor if CAN messages stop appearing
    motor.disable();
    return;
  }

  else if (!canControl.isEnabled()) { // If should be disabled, disable motor
    motor.disable();

    // CAN status only every 10ms
    if (millis() - last_status_time >= STATUS_INTERVAL) {
        canControl.sendStatus(sensor.getAngle(), sensor.getVelocity(), currentSense.getDCCurrent());
        last_status_time = millis();
    }
  } 

  else
  { 
    if (motor.enabled == false) { // If all is good, move motor
      motor.enable();
    }

    float target = canControl.getTargetValue();
    motor.move(target);

    // CAN status only every 10ms
    if (millis() - last_status_time >= STATUS_INTERVAL) {
        canControl.sendStatus(sensor.getAngle(), sensor.getVelocity(), currentSense.getDCCurrent());
        last_status_time = millis();
    }
  }
}