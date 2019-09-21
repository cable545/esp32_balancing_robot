#include <Wire.h>
#include "AdafruitMpu.h"
#include "esp32-hal-cpu.h"
#include "MyStepper.h"
#include "Pid.h"
#include "config.h"

#define I2Cclock 400000

uint32_t cpuClock;
uint32_t timeMeasure = 0, lastTimeMeasure = 0;
float deltaT = 0;
int ledPin = 13;
bool ledToggle = false;
float addTime;

AdafruitMpu mpu;
ImuData_t gyroData;
EulerData_t accEulerData, gyroEulerData;
uint32_t rollAngle;

MyStepper& leftStepper = MyStepper::getLeftStepper();
MyStepper& rightStepper = MyStepper::getRightStepper();

Pid anglePID(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_I_LIMIT);

void setup()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  Serial1.begin(115200);
 
  Wire.begin();
  
  searchForDevices();

  if(mpu.init())
    Serial1.println("Mpu initialized");
  else
    Serial1.println("Mpu initialization failed");

  //cpuClock = getCpuFrequencyMhz(); //Get CPU clock
/*
  leftStepper.stepperTimerStart();
  leftStepper.setTimerValue(130);
  rightStepper.stepperTimerStart();
  rightStepper.setTimerValue(130);
  */
}

void loop()
{
  mpu.accRead();
  mpu.gyroRead();
  mpu.accCalcAngles();
  rollAngle = mpu.calculateRollAngle(deltaT);

  float motorSpeed = anglePID.updatePID(0.0, rollAngle, deltaT);

  Serial1.print(rollAngle); Serial1.print("  "); Serial1.print(motorSpeed); Serial1.println();

  timeMeasure = millis();
  deltaT = (float)(timeMeasure - lastTimeMeasure) / 1000.0;
  addTime += deltaT;
  lastTimeMeasure = timeMeasure;

  if(addTime >= 1.0)
  {
    digitalWrite(ledPin, ledToggle);
    ledToggle = !ledToggle;
    addTime = 0;
  }
}

void searchForDevices()
{  
  uint8_t cnt = 0;

  Serial1.println("Scanning I2C Addresses");
  
  for(uint8_t i = 0; i < 128; i++)
  {
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission();
    if(ec == 0)
    {
      if(i < 16)Serial1.print('0');
      Serial1.print(i, HEX);
      cnt++;
    }
    else 
    {
      Serial1.print("..");
    }
    
    Serial1.print(' ');
    if ((i & 0x0f) == 0x0f)
      Serial1.println();
  }

  Serial1.print("Scan Completed, ");
  Serial1.print(cnt);
  Serial1.println(" I2C Devices found.");
}
