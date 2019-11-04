#include "Sonar.h"


Sonar::Sonar(uint8_t address)
{
  i2cAddress = address;
}


uint32_t Sonar::readSensor()
{
 return 0;  
}
  
bool Sonar::init()
{
  return true;
}
