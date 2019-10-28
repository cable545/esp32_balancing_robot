#include "Arduino.h"
#include "Pid.h"

Pid::Pid(float p, float i, float d, float iLimit)
{
  P = p;
  I = i;
  D = d;
  I_limit = iLimit;
  integratedError = 0;
  lastError = 0;
}

float Pid::updatePID(float target, float current, float deltaTime)
{
  float error = (target - current) * deltaTime; 
  
  float pPart = P * error;
  
  integratedError += error;    
  integratedError = constrain(integratedError, -I_limit, I_limit);
  float iPart = I * integratedError;
  
  float dPart = D * (error - lastError);    
  
  lastError = error;
   
  return (pPart + iPart + dPart); 
}

void Pid::resetPID()
{
  integratedError = 0; 
  lastError = 0; 
}

void Pid::setGains(float p, float i, float d)
{
  
}
