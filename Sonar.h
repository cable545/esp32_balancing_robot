#ifndef _SONAR_H
#define _SONAR_H

#include <stdint.h>

#define SONAR_I2C_CLK 400000

class Sonar
{
  public:
    bool init();
    uint32_t read();
};

#endif
