#include "Arduino.h"
/**************************************************************************************
* Copyright (C) 2024 JMKO
* 
* Author: Jonathan Olund
*Pins from bottom lens pointing to right
*	Lidar		328p      Micro/pro
*	Vcc	  	5v        Vcc
*	Gnd		  Gnd       Gnd
*	SCL		  SCL(ADC5) 2 
*	SDA		  SDA(ADC4) 3
*	Mode	  Gnd       Gnd
***************************************************************************************/
#pragma once
#include <Wire.h>
#include "sen0585.h"
int nDevices = 1;
uint16_t timeout_count = 0;
uint16_t readlimit = 0;
uint16_t dist[2];
int inch;
int sensAddr[2] = {0x10, 0x11};

/*************************kalman Filter*******************************/
bool filters_klm_on = true;
uint16_t klm_factor = 300;
uint16_t klm_threshold = 300;
int last_time = 0;
uint16_t last_distance = 0;
/*************************kalman Filter*******************************/
uint16_t doKlmFilter(uint16_t & distance)
{
	if(filters_klm_on)
	{			
		bool bfilterstart = false;
		int currentTime = millis();        
		if(abs(currentTime - last_time)<300)//300ms
			bfilterstart = true;
		last_time = currentTime;

		if((distance < 64000)&&(last_distance < 64000)&& bfilterstart)
		{
			int32_t diff = distance - last_distance;

			if (abs(diff) < klm_threshold)
			{
				uint32_t result = 0;
				result = (((distance * klm_factor) + (last_distance * (1000-klm_factor))) / 1000);
				distance = result;
			}
		}			
		last_distance = distance;			
	}
}
/********************************************************/


uint8_t distance(int sensorNum) {
  //uint8_t data[10];
  
  //(set register address)
  Wire.beginTransmission(sensAddr[sensorNum]);
  Wire.write(byte(23));
  Wire.write(byte(0));
  Wire.endTransmission();
  
  Wire.requestFrom(sensAddr[sensorNum], 2);//请求从设备读取2个字节的数据（Request to read 2 bytes of data from the device）

  if (Wire.available() >= 2) {
    int lowByte = Wire.read();   //Read low byte
    int highByte = Wire.read();  //Read high byte
    uint16_t distance = (highByte << 8) | lowByte;
    
    if(nDevices > 0)
    {
      timeout_count = 0;
      Serial.print(nDevices);
      Serial.print(" ");
      Serial.println("connected");

      doKlmFilter(distance);
      if(distance < 64000)
      { 
        inch = distance/25;
        Serial.print(inch);
        Serial.println(" on left");
      }else
      {
        Serial.print("ErrorCode:");
        Serial.println(distance);
      }
      
    dist[sensorNum] = inch;
    }
  }
return inch;
}

uint16_t sens_readReg(int sens, uint8_t regaddr)
{
  int LSB;
  int MSB;
  Wire.beginTransmission(sensAddr[sens]);
  Wire.write(byte(regaddr));
  //Wire.write(byte(0));
  Wire.endTransmission();
  
  Wire.requestFrom(sens, 2);
  delay(200);
  if (Wire.available() >=2){
  LSB = Wire.read();
  MSB = Wire.read();
  }
  uint16_t output = ((MSB << 8) | LSB);
  
  //Serial.print(output,HEX);
  return output;
}

uint16_t sens_writeReg(int sens, uint8_t regaddr, uint8_t value)
{
  Wire.beginTransmission(sensAddr[sens]);
  Wire.write(byte(regaddr));
  Wire.write(byte(0));
  Wire.write(byte(value & 0xFF));
  Wire.write(byte(value >> 8));
  Wire.endTransmission();
  //Wire.requestFrom(sensAddr[sens],2);
  //byte LSB = Wire.read();
  //byte MSB = Wire.read();
  //uint8_t output = ((MSB << 8) | LSB);
  //Serial.print("Wrote: 0x");
  //Serial.print(output, HEX);
  //Serial.print(" to ");
  //Serial.println(regaddr);
  return Wire.endTransmission();
}

void startDist(int sens)
{
  sens_writeReg(sens, 0, 0); //电流寄存器设置为0(Set the current register to 0)
  sens_writeReg(sens, 61, 0x000); //设置为主动读取结果 (Set the outgoing mode to 0x0000 for active reading result mode).
  sens_writeReg(sens, 66, 200);//周期寄存器 (Set the measurement cycle register to 200ms)
  sens_writeReg(sens, 3, 0x0001); //start measuring
}
void stopDist(int sens)
{
  sens_writeReg(sens, 0, 0); //电流寄存器设置为0(Set the current register to 0)
  sens_writeReg(sens, 61, 0x000); //设置为主动读取结果 (Set the outgoing mode to 0x0000 for active reading result mode).
  sens_writeReg(sens, 66, 200);//周期寄存器 (Set the measurement cycle register to 200ms)
  sens_writeReg(sens, 3, 0x0002); //stop measuring
}

uint16_t initdev(int SensEnable) {
  Serial.print("checking: 0x");
  Serial.println(sensAddr[0],HEX);
  sen0585_init(); // Initialize I2C communication with SEN0585
  
  // Read the 16-bit value from the register
  uint16_t value = sen0585_read16BitRegister(SEN0585_DISTANCE_REGISTER);
  Serial.print("Original value: 0x");
  Serial.println(value, HEX); // Print original value in hexadecimal

  // Increment the low byte by 0x01
  uint16_t newValue = sen0585_incrementLowByte(value);
  Serial.print("New value: 0x");
  Serial.println(newValue, HEX); // Print updated value in hexadecimal

  // Write the new value back to the register
  sen0585_write16BitRegister(SEN0585_DISTANCE_REGISTER, newValue);
  Serial.println(" init done");
  return nDevices;
}
