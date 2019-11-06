#include "Sonar.h"

#include <Wire.h>

#define SONAR_I2C_ADDRESS 0x70 //111 0000
#define RANGE_READING_ADDRESS  0x51 //0101 0001

uint32_t Sonar::read()
{
  uint8_t lowByte, highByte;
  uint32_t range;

  Wire.beginTransmission(SONAR_I2C_ADDRESS);
  Wire.write(RANGE_READING_ADDRESS);
  Wire.endTransmission();

  delay(100);
  
  Wire.requestFrom(SONAR_I2C_ADDRESS, (uint8_t)2);
  highByte = Wire.read();
  lowByte = Wire.read();
  Wire.endTransmission();

  range = (highByte << 8) | lowByte;

  return range;
}

bool Sonar::init()
{
  Wire.begin();
  Wire.setClock(SONAR_I2C_CLK);
  
  return true;
}
