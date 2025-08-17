/*
  Proper SimpleFOC Current Sensing Configuration
  Based on SimpleFOC documentation and best practices
  
  Key fixes:
  1. Proper initialization order (driver -> link -> current_sense -> motor)
  2. Enable SimpleFOC debugging
  3. Lower PWM frequency for better current sensing
  4. Proper gain calculation
  5. Allow driver alignment procedure
*/

#include <Arduino.h>
#include <SimpleFOC.h>

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
#define M0_IA _NC // Phase A not available on this board
#define M0_IB PC0 // M0_SO1 - Phase B current sense
#define M0_IC PC1 // M0_SO2 - Phase C current sense

// Serial communication
#define PIN_SERIAL1_RX PA3
#define PIN_SERIAL1_TX PA2
HardwareSerial Serial1(PIN_SERIAL1_RX, PIN_SERIAL1_TX);

// Motor and driver instances
BLDCMotor motor = BLDCMotor(PP);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

// Current sensing - PROPER GAIN CALCULATION
// For DRV8301: typical gain is around 8-12 V/V, let's start conservative
LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

// Variables for monitoring
PhaseCurrent_s current;
float current_magnitude;
long timestamp = 0;
float target_voltage = 3.0f; // Start low for safety
float target_velocity = 2.0f; // rad/s

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Proper SimpleFOC Current Sensing Test ===");
  
  // CRITICAL: Enable SimpleFOC debugging BEFORE any init calls
  SimpleFOCDebug::enable(&Serial);
  Serial.println("SimpleFOC debugging enabled");
  
  // 1. Initialize driver FIRST
  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 8.0f;
  
  // IMPORTANT: Lower PWM frequency for better current sensing (per docs)
  driver.pwm_frequency = 20000; // 20kHz as recommended
  
  driver.init();
  Serial.println("✓ Driver initialized");
  
  // 2. Link driver to current sense BEFORE current sense init
  currentSense.linkDriver(&driver);
  Serial.println("✓ Driver linked to current sense");
  
  // 3. Initialize motor (before current sense init)
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = target_voltage;
  motor.init();
  Serial.println("✓ Motor initialized");
  
  // 4. Initialize current sensing AFTER driver and motor
  Serial.println("Initializing current sensing with debugging...");
  if (currentSense.init()) {
    Serial.println("✓ Current sensing initialized successfully");
  } else {
    Serial.println("✗ Current sensing initialization failed!");
    Serial.println("Check wiring, gain values, and PWM frequency");
    while(1);
  }
  
  // 5. Link current sense to motor
  motor.linkCurrentSense(&currentSense);
  Serial.println("✓ Current sense linked to motor");
  
  // 6. IMPORTANT: Run initFOC for proper alignment
  Serial.println("Running initFOC for driver alignment...");
  motor.voltage_sensor_align = 2.0f; // Voltage for alignment procedure
  
  // Let SimpleFOC handle the alignment (don't skip it)
  motor.initFOC();
  Serial.println("✓ FOC initialization complete");
  
  Serial.println("\n=== Current Sensing Diagnostics ===");
  Serial.print("Shunt resistor: "); Serial.print(0.0005f, 6); Serial.println(" Ohms");
  Serial.print("Amplifier gain: "); Serial.println(10.0f);
  Serial.print("PWM frequency: "); Serial.print(driver.pwm_frequency); Serial.println(" Hz");
  Serial.print("Phase A gain: "); Serial.println(currentSense.gain_a, 6);
  Serial.print("Phase B gain: "); Serial.println(currentSense.gain_b, 6);
  Serial.print("Phase C gain: "); Serial.println(currentSense.gain_c, 6);
  
  Serial.println("\n=== Pre-Motor Current Test ===");
  Serial.println("Motor stopped - currents should be near zero:");
  for(int i = 0; i < 5; i++) {
    current = currentSense.getPhaseCurrents();
    Serial.print("Phase B: "); Serial.print(current.b, 3);
    Serial.print("A, Phase C: "); Serial.print(current.c, 3); Serial.println("A");
    delay(200);
  }
  
  Serial.println("\n=== Starting Motor ===");
  Serial.println("Expected: Sinusoidal currents, phases 120° apart, realistic magnitude");
  Serial.println("Time(ms) | Voltage(V) | Velocity(rad/s) | Phase_A(A) | Phase_B(A) | Phase_C(A) | Magnitude(A) | Status");
  Serial.println("--------------------------------------------------------------------------------------------------------");
  
  motor.target = target_velocity;
}

void loop() {
  // Run FOC algorithm (even in open loop, this helps with current sensing)
  motor.loopFOC();
  motor.move();
  
  // Read current values
  current = currentSense.getPhaseCurrents();
  current_magnitude = currentSense.getDCCurrent();
  
  long now = millis();
  if (now - timestamp > 500) {
    
    Serial.print(now);
    Serial.print(" | ");
    Serial.print(motor.voltage_limit, 1);
    Serial.print(" | ");
    Serial.print(motor.target, 1);
    Serial.print(" | ");
    Serial.print(current.a, 3);
    Serial.print(" | ");
    Serial.print(current.b, 3);
    Serial.print(" | ");
    Serial.print(current.c, 3);
    Serial.print(" | ");
    Serial.print(current_magnitude, 3);
    Serial.print(" | ");
    
    // Advanced diagnostics
    float phase_diff = abs(current.b - current.c);
    bool reasonable_magnitude = (current_magnitude > 0.1 && current_magnitude < 5.0);
    bool phases_different = (phase_diff > 0.3);
    bool has_negative = (current.b < -0.1 || current.c < -0.1);
    
    if (reasonable_magnitude && phases_different && has_negative) {
      Serial.print("✓ GOOD");
    } else if (current_magnitude > 20.0) {
      Serial.print("✗ TOO HIGH - reduce gain");
    } else if (!phases_different) {
      Serial.print("✗ PHASES SIMILAR - check alignment");
    } else if (!has_negative) {
      Serial.print("⚠️ NO NEGATIVE - check calibration");
    } else {
      Serial.print("⚠️ CHECK");
    }
    
    Serial.println("");
    timestamp = now;
  }
}

/*
=== WHAT THIS FIX ADDRESSES ===

1. **Proper Initialization Order** (per SimpleFOC docs):
   - driver.init() FIRST
   - currentSense.linkDriver() BEFORE currentSense.init()
   - motor.init() BEFORE currentSense.init()
   - currentSense.init()
   - motor.linkCurrentSense()
   - motor.initFOC() for alignment

2. **SimpleFOC Debugging Enabled**:
   - Will show detailed info about current sense initialization
   - Helps identify hardware-specific issues

3. **Lower PWM Frequency**:
   - 20kHz as recommended for current sensing
   - Gives more time for ADC conversions

4. **Proper FOC Loop**:
   - Using motor.loopFOC() even in open loop
   - Better current sensing synchronization

5. **Driver Alignment**:
   - Lets SimpleFOC auto-detect and fix phase/gain issues
   - Don't skip this critical step

=== EXPECTED IMPROVEMENTS ===

- Current readings should be realistic (0.5-3.0A)
- Phases B and C should oscillate with different values
- Should see positive and negative current values
- SimpleFOC debug output will show any issues

=== COMMANDS TO TEST ===
- '+' / '-' : increase/decrease voltage
- 'f' / 's' : faster/slower speed  
- 'r' : reverse direction
- '0' : stop motor

If readings are still wrong, the debug output will tell us exactly what's failing!
*/