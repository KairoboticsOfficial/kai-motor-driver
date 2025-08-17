/*
  SimpleFOC MKS DRIVE MINI + AS5047P
  SimpleFOC_STM32F405RGT6

  Odrive 3.6 ==> Odesk 4.2
  https://github.com/makerbase-motor/ODrive-MKS/tree/main/Hardware

  AS5047P
  https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/as5047

  Makerbase-Mini contrôleur de servomoteur sans balais XDrive, haute précision, basé sur ODriLi3.6 avec AS5047P à bord
  https://fr.aliexpress.com/item/1005006480243178.html
*/

#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"

//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬AS5047_SPI▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
// magnetic sensor instance - SPI
#define CS PA15 // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10 // PC10

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

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin
// MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs)
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, CS);
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, CS);
//MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF); // // alternative constructor (chipselsect, bit_resolution, angle_read_register, )

// https://github.com/simplefoc/Arduino-FOC/blob/ee2dfdeee62bc28fdee5820fb1d26a0af4dc80c9/src/sensors/MagneticSensorSPI.cpp
// https://github.com/simplefoc/Arduino-FOC/blob/master/examples/utils/sensor_test/magnetic_sensors/magnetic_sensor_spi/magnetic_sensor_spi_alternative_examples/stm32_spi_alt_example/stm32_spi_alt_example.ino
// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
// SPIClass SPI_2(mosi, miso, sclk);
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI0SCK);

float Angle;
float Velocity;

void setup() {
  // initialise magnetic sensor hardware
  //sensor.clock_speed = 500000;
  sensor.init(&SPI_2);
  _delay(2000);
} // End setup

void loop() {
  sensor.update();
  // Read variables on STMViewer and St-Link V2
  Serial.println(sensor.getAngle()*1.0);
  _delay(1);
  Velocity = sensor.getVelocity();
} // End loop