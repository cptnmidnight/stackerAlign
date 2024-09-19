/**************************************************************************************
* Copyright (C) 2024 JMKO
* 
* Author: Jonathan Olund
*Pins from bottom laser pointing to right
*	Lidar		328p      Micro/pro
*	Vcc	  	5v        Vcc
*	Gnd		  Gnd       Gnd
*	SCL		  SCL(ADC5) 2 
*	SDA		  SDA(ADC4) 3
*	Mode	  Gnd       Gnd
***************************************************************************************/
#include "xts1_functions.hpp"

int sens1M = 21; //i2c sensor 1
int sens2M = 20; //i2c sensor 2
int sens1P = 19; //power sensor1
int sens2P = 18; //power sensor2
int lvlH = 8;
int lvlG = 9;
int sens2G = 10;


void setup() {
pinMode(sens1M, OUTPUT);
pinMode(sens2M, OUTPUT);
pinMode(sens1P, OUTPUT);
pinMode(sens2P, OUTPUT);
digitalWrite(sens1M, LOW);
digitalWrite(sens2M, LOW);
digitalWrite(sens1P, HIGH);
digitalWrite(sens2P, LOW);
pinMode(lvlH, OUTPUT);
pinMode(lvlG, OUTPUT);
pinMode(sens2G, OUTPUT);
digitalWrite(lvlH,HIGH);
digitalWrite(lvlG, LOW);
digitalWrite(sens2G, LOW);

Serial.begin(115200);
while (!Serial);
Serial.println("Serial on");
Wire.begin();
Serial.println("I2C on");
//Serial.println("wait connect");
initdev(sens2P);
//Serial.println(distance(0));
}

void loop()
{
  if (nDevices > 0)
  {
  //distance(0);
//Serial.print("sensor0 ");  
//Serial.println(dist[0]);
 // delay(5000);
//  distance(1);
//Serial.print("sensor1 ");  
//Serial.println(dist[1]);
 // delay(5000);
  }
}
