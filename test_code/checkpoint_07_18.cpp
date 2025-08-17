/*
  Torque Control Test with Current Monitoring
  SimpleFOC MKS DRIVE MINI + AS5047P + Motor
  
  This controls motor torque (current) directly
  Perfect for testing stable current control
*/

#include <Arduino.h>
#include <SimpleFOC.h>

// Motor parameters
#define PP 14  // Your motor's pole pairs

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
#define M0_IB PA0 // Phase B current sense
#define M0_IC PA1 // Phase C current sense

// Encoder pins
#define CS PA15
#define SPI_MISO PC11
#define SPI_MOSI PC12
#define SPI0SCK PC10

// Serial communication
#define PIN_SERIAL1_RX PA3
#define PIN_SERIAL1_TX PA2
HardwareSerial Serial1(PIN_SERIAL1_RX, PIN_SERIAL1_TX);

// Motor, driver, and sensor instances
BLDCMotor motor = BLDCMotor(PP);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

// Encoder setup - Using your working AS5047P configuration
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
float target_current = 1.0f; // Start with very low current

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Torque Control Test ===");
  
  // Initialize encoder - using your working setup
  sensor.init(&SPI_2);
  motor.linkSensor(&sensor);
  Serial.println("✓ Encoder initialized");
  
  // Initialize driver
  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 24.0f;
  driver.init();
  motor.linkDriver(&driver);
  Serial.println("✓ Driver initialized");
  
  // Initialize current sensing
  currentSense.linkDriver(&driver);
  if (currentSense.init()) {
    Serial.println("✓ Current sensing initialized");
  } else {
    Serial.println("✗ Current sensing failed!");
    while(1);
  }
  
  // Configure motor for TORQUE control
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  
  motor.PID_current_q.P = 1.3f;    // Much lower
  motor.PID_current_q.I = 0.0f;   // Much lower  
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_q.output_ramp = 2000.0f;
  motor.LPF_current_q.Tf = 0.02f;

  //▬▬▬▬▬▬▬▬▬▬▬▬current d loop PID
  motor.PID_current_d.P = 1.3f; // 3
  motor.PID_current_d.I = 0.0f; // 300
  motor.PID_current_d.D = 0.0f;
  motor.PID_current_d.output_ramp = 2000.0f;
  motor.LPF_current_d.Tf = 0.02f;  // Low pass filtering time constant
  
  // Motor limits - Conservative settings
  motor.voltage_limit = 20.0f;
  motor.current_limit = 15.0f;
  
  // Link current sensing
  currentSense.skip_align = false;
  motor.linkCurrentSense(&currentSense);
  
  // Initialize motor
  motor.init();
  motor.initFOC();
  
  Serial.println("\n=== Starting Torque Control Test ===");
  Serial.print("Target current: "); Serial.print(target_current); Serial.println("A");
  Serial.println("\nTime(ms) | Target(A) | Phase_A(A) | Phase_B(A) | Phase_C(A) | Magnitude(A) | Speed(rad/s)");
  Serial.println("---------------------------------------------------------------------------------------------");
  
  // Set target current
  motor.target = target_current;
}

void loop() {
  // Run FOC control loop
  motor.loopFOC();
  motor.move();
  
  // Read current values
  current = currentSense.getPhaseCurrents();
  current_magnitude = currentSense.getDCCurrent();
  
  // Print readings every 100ms
  long now = millis();
  if (now - timestamp > 100) {
    
    Serial.print(now);
    Serial.print(" | ");
    Serial.print(motor.target, 2);
    Serial.print(" | ");
    Serial.print(current.a, 3);
    Serial.print(" | ");
    Serial.print(current.b, 3);
    Serial.print(" | ");
    Serial.print(current.c, 3);
    Serial.print(" | ");
    Serial.print(current_magnitude, 3);
    
    // Status indicators
    if (abs(current_magnitude - motor.target) < 0.2) {
      Serial.print(" ✓ Good tracking");
    }
    if (current_magnitude > 4.0) {
      Serial.print(" ⚠️ High current");
    }
    
    Serial.println("");
    timestamp = now;

    sensor.update();
    // Read variables on STMViewer and St-Link V2
    Serial.println(sensor.getAngle()*1.0);
  }
  
}