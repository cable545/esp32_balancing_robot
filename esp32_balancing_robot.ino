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
float rollAngle;

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

  leftStepper.stepperTimerStart();
  leftStepper.setTimerValue(512000);
  rightStepper.stepperTimerStart();
  rightStepper.setTimerValue(512000);

}

void loop()
{
  mpu.accRead();
  mpu.gyroRead();
  mpu.accCalcAngles();
  rollAngle = mpu.calculateRollAngle(deltaT);
  
  float motorSpeed = anglePID.updatePID(0.0, rollAngle, deltaT);

  motorSpeed *= 3600;

  /*
  leftStepper.setDirection(LEFT_STEPPER_FORWARD_DIR_LEVEL);
  rightStepper.setDirection(RIGHT_STEPPER_FORWARD_DIR_LEVEL);

  leftStepper.setTimerValue(MIN_TIMER_PERIOD);
  rightStepper.setTimerValue(MIN_TIMER_PERIOD);
*/

  leftStepper.setMotorSpeed(motorSpeed);
  rightStepper.setMotorSpeed(motorSpeed);

  //Serial.print(mpu.getGyroData().x); Serial.print("  ");
  
  //Serial.print(rollAngle); Serial.print("  "); Serial.print(motorSpeed);
  //Serial.println();

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

  Serial.println("Scanning I2C Addresses");
  
  for(uint8_t i = 0; i < 128; i++)
  {
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission();
    if(ec == 0)
    {
      if(i < 16)Serial.print('0');
      Serial.print(i, HEX);
      cnt++;
    }
    else 
    {
      Serial.print("..");
    }
    
    Serial.print(' ');
    if ((i & 0x0f) == 0x0f)
      Serial.println();
  }

  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");
}
