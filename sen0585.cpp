#include "sen0585.h"

// Initialize I2C communication with the SEN0585
void sen0585_init() {
  Wire.begin(); // Initialize I2C
}

// Read a 16-bit register from the SEN0585
uint16_t sen0585_read16BitRegister(uint8_t reg) {
  Wire.beginTransmission(SEN0585_ADDRESS); // Start communication with device
  Wire.write(reg); // Specify the register address
  Wire.endTransmission(false); // End transmission but keep connection open
  
  Wire.requestFrom(SEN0585_ADDRESS, 2); // Request 2 bytes from the device
  
  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read(); // Read high byte
    uint8_t lowByte = Wire.read();  // Read low byte
    return (highByte << 8) | lowByte; // Combine high and low bytes into a 16-bit value
  } else {
    return 0; // Return 0 if not enough data available
  }
}

// Write a 16-bit value to a register in the SEN0585
void sen0585_write16BitRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(SEN0585_ADDRESS); // Start communication with device
  Wire.write(reg); // Specify the register address
  Wire.write((value >> 8) & 0xFF); // Write high byte
  Wire.write(value & 0xFF); // Write low byte
  Wire.endTransmission(); // End transmission
}

// Increment the low byte of a 16-bit value by 0x01
uint16_t sen0585_incrementLowByte(uint16_t value) {
  uint8_t highByte = (value >> 8) & 0xFF; // Extract high byte
  uint8_t lowByte = value & 0xFF; // Extract low byte
  
  lowByte += 0x01; // Increment low byte
  
  // Ensure low byte stays within 8-bit range (0x00 to 0xFF)
  if (lowByte > 0xFF) {
    lowByte = 0x00; // Wrap around if overflow
  }
  
  return (highByte << 8) | lowByte; // Combine high byte and modified low byte
}

