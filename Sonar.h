#ifndef _SONAR_H
#define _SONAR_H

#define SONAR_I2C_ADDRESS 0x70 //111 0000 

class Sonar
{
  /*
  public:
    //Sonar(uint8_t address);
    //uint32_t readSensor();
    
  private:
    void init();
  */
    uint8_t i2cAddress;
    
};

#endif
