#ifndef SEN0585_H
#define SEN0585_H

#include <Wire.h>

// I2C address of the SEN0585
#define SEN0585_ADDRESS 0x10

// Register address for the distance data (replace if different)
#define SEN0585_DISTANCE_REGISTER 0x80

// Function to initialize I2C communication with the SEN0585
void sen0585_init();

// Function to read a 16-bit register from the SEN0585
uint16_t sen0585_read16BitRegister(uint8_t reg);

// Function to write a 16-bit value to a register in the SEN0585
void sen0585_write16BitRegister(uint8_t reg, uint16_t value);

// Function to increment the low byte of a 16-bit value
uint16_t sen0585_incrementLowByte(uint16_t value);

#endif // SEN0585_H

