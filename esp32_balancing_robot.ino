#include <Wire.h>
#include <BluetoothSerial.h>

#include "esp32-hal-cpu.h"

#include "AdafruitMpu.h"
#include "MyStepper.h"
#include "Pid.h"
#include "Cmd.h"
#include "Config.h"

#define I2Cclock 400000

uint32_t cpuClock;
uint32_t timeMeasure = 0, lastTimeMeasure = 0;
float deltaT = 0;
int ledPin = 13;
bool ledToggle = false;
float ledAddTime, stablePositionTime;

AdafruitMpu mpu;
ImuData_t gyroData;
EulerData_t accEulerData, gyroEulerData;
float rollAngle;

Config& myConfig = Config::getConfig();

MyStepper& leftStepper = MyStepper::getLeftStepper();
MyStepper& rightStepper = MyStepper::getRightStepper();

Pid anglePID(
  myConfig.getPGainAngle(),
  myConfig.getIGainAngle(),
  0.0,
  myConfig.getILimitAngle()
);

TaskHandle_t commTask;



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

  xTaskCreatePinnedToCore(
    commTaskHandler,
    "CommTask",
    10000,
    NULL,
    1,
    &commTask,
    0
  );
}

void loop()
{
  /*
  mpu.accRead();
  mpu.gyroRead();
  mpu.accCalcAngles();
  rollAngle = mpu.calculateRollAngle(deltaT);

  if(rollAngle > 45.0 || rollAngle < -45.0)
  {
    stablePositionTime = 0;
    anglePID.resetPID();
    leftStepper.disableInterrupt();
    rightStepper.disableInterrupt();
  }
  else
  {
    if(stablePositionTime >= 3.0)
    {
      leftStepper.enableInterrupt();
      rightStepper.enableInterrupt();
      float motorSpeed = anglePID.updatePID(0.0, rollAngle, deltaT);
      motorSpeed *= 3600;
      leftStepper.setMotorSpeed(motorSpeed);
      rightStepper.setMotorSpeed(motorSpeed);
    }
  }

  //Serial.print(mpu.getGyroData().x); Serial.print("  ");
  
  //Serial.print(rollAngle); Serial.print("  "); Serial.print(motorSpeed);
  //Serial.println();
*/
  timeMeasure = millis();
  deltaT = (float)(timeMeasure - lastTimeMeasure) / 1000.0;
  ledAddTime += deltaT;
  stablePositionTime += deltaT;
  lastTimeMeasure = timeMeasure;

  if(ledAddTime >= 1.0)
  {
    digitalWrite(ledPin, ledToggle);
    ledToggle = !ledToggle;
    ledAddTime = 0;
  }
}

void commTaskHandler(void * pvParameters)
{
  BluetoothSerial btInstance;
  btInstance.begin("ESP32-BalancingRobot");
  
  for(;;)
  {
    if(btInstance.available())
    {
      Cmd::requestHandler(btInstance.read());
      
      //int value = btInstance.read();
      //Serial1.print("Data: "); Serial1.print(value); Serial1.println();
    }
    else
    {
      delay(10);
    }
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
