#include <Wire.h>
#include "AdafruitMpu.h"

#define I2Cclock 400000

uint32_t timeMeasure = 0, lastTimeMeasure = 0;
float deltaT = 0;
int ledPin = 13;
AdafruitMpu mpu;
ImuData_t gyroData;
EulerData_t accEulerData, gyroEulerData;

float pitch;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  pinMode(ledPin, OUTPUT);
  searchForDevices();

  if(mpu.init())
    Serial.println("Mpu initialized");
  else
    Serial.println("Mpu initialization failed");
}

void loop()
{
  mpu.accRead();
  mpu.gyroRead();
  mpu.accCalcAngles();
  pitch = mpu.calculatePitchAngle(deltaT);

//  Serial.print(gyroEulerData.roll);Serial.print(';');
  Serial.println((int)pitch);
  
  timeMeasure = millis();
  deltaT = (float)(timeMeasure - lastTimeMeasure) / 1000.0;
  lastTimeMeasure = timeMeasure;
  
//  digitalWrite(ledPin, HIGH);
//  delay(500);
//  digitalWrite(ledPin, LOW);
//  delay(500);
  
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
