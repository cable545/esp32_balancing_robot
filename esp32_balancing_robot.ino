#include <Wire.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

#include <rom/rtc.h>

#include "string.h"

#include "esp32-hal-cpu.h"

#include "AdafruitMpu.h"
#include "MyStepper.h"
#include "Pid.h"
#include "Cmd.h"
#include "Config.h"
#include "Sonar.h"
#include "Servo_ESP32.h"

#define Serial Serial
#define I2Cclock 400000

int servoPin = 26;

Servo_ESP32 servo1;

int angle = 0;
int angleStep = 5;

int angleMin = 0;
int angleMax = 180;

uint32_t cpuClock, i;
uint32_t timeMeasure = 0, timeMeasure2 = 0, timeMeasure3 = 0, lastTimeMeasure = 0, elapsedMicros = 0;
float deltaT = 0;
int ledPin = 13;
bool ledToggle = false;
float ledAddTime, stablePositionTime;

QueueHandle_t messageQueue;
int queueSize = 4;

AdafruitMpu mpu;
ImuData_t gyroData;
EulerData_t accEulerData, gyroEulerData;
float rollAngle;

MyStepper& leftStepper = MyStepper::getLeftStepper();
MyStepper& rightStepper = MyStepper::getRightStepper();

TaskHandle_t commTask;
Sonar sonar;

void setup()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  Serial1.begin(115200);
  Serial.begin(115200);
  Serial.println("initialized");

  servo1.attach(servoPin);

  //cpuClock = getCpuFrequencyMhz(); //Get CPU clock
  searchForDevices();

  //sonar.init();

  ledcAttachPin(servoPin, 0);
  messageQueue = xQueueCreate(queueSize, sizeof(float));

  if(messageQueue == NULL){
    Serial.println("Error creating the message queue");
  }

  if(mpu.init())
    Serial.println("Mpu initialized");
  else
    Serial.println("Mpu initialization failed");

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

  Serial.println("end setup");
}

void loop()
{
  ImuData_t gyroOffsets, accOffsets;
  uint32_t error = 0;
  
  //uint32_t range = sonar.read();
  // float motorSpeed = 0.0;

  //Serial.print("R");Serial.print(i++);Serial.print(": ");Serial.println(range);
  
  PidGainContainer pidGainContainer;

  while(true)
  {
    if(uxQueueMessagesWaiting(messageQueue) == 4)
      break;
  }
  
  xQueueReceive(messageQueue, &pidGainContainer.pGainAngle, portMAX_DELAY);
  xQueueReceive(messageQueue, &pidGainContainer.iGainAngle, portMAX_DELAY);
  xQueueReceive(messageQueue, &pidGainContainer.dGainAngle, portMAX_DELAY);
  xQueueReceive(messageQueue, &pidGainContainer.iLimitAngle, portMAX_DELAY);
  
  Pid anglePID(
    pidGainContainer.pGainAngle,
    pidGainContainer.iGainAngle,
    pidGainContainer.dGainAngle,
    pidGainContainer.iLimitAngle
  );

  int posDegrees = 0;
  int up = 1;
  
  while(true)
  {
    
    
    for(int posDegrees = 0; posDegrees < 180; posDegrees++)
    {
      servo1.write(posDegrees);
      //Serial.println(posDegrees);
      delay(2);
    }

    for(posDegrees = 179; posDegrees > 0; posDegrees--)
    {
      servo1.write(posDegrees);
      //Serial.println(posDegrees);
      delay(2);
    }
    
    /*
    mpu.accRead();
  
    Serial.print(mpu.getAccData().x, 6);Serial.print(" ");
    Serial.print(mpu.getAccData().y, 6);Serial.print(" ");
    Serial.println(mpu.getAccData().z, 6);

    mpu.gyroReadX();
    Serial.println(mpu.getGyroData().x, 6);

    mpu.accReadComp();
    mpu.gyroReadXComp();
    mpu.accCalcAngles();
    rollAngle = mpu.calculateRollAngle(deltaT);
    Serial.println(rollAngle, 3);
    
    mpu.executeCalibration(&gyroOffsets, &accOffsets);
    Serial.println("Gyro");
    Serial.println(gyroOffsets.x, 6);

    Serial.println("Acc");
    Serial.print(accOffsets.x, 6);Serial1.print(" ");
    Serial.print(accOffsets.y, 6);Serial1.print(" ");
    Serial.println(accOffsets.z, 6);
    
    
    mpu.accReadComp();
    mpu.gyroReadXComp();
    mpu.accCalcAngles();
    rollAngle = mpu.calculateRollAngle(deltaT);

    Serial.print(rollAngle, 3);Serial.print("   ");Serial.println();

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
        //float targetAngle = speedPID.updatePID(0.0, motorSpeed); 
        float motorSpeed = anglePID.updatePID(0.0, rollAngle);
        motorSpeed *= 3200;
        leftStepper.setMotorSpeed(motorSpeed);
        rightStepper.setMotorSpeed(motorSpeed);
      }
    }

    if(uxQueueMessagesWaiting(messageQueue) == 4)
    {
      xQueueReceive(messageQueue, &pidGainContainer.pGainAngle, portMAX_DELAY);
      xQueueReceive(messageQueue, &pidGainContainer.iGainAngle, portMAX_DELAY);
      xQueueReceive(messageQueue, &pidGainContainer.dGainAngle, portMAX_DELAY);
      xQueueReceive(messageQueue, &pidGainContainer.iLimitAngle, portMAX_DELAY);
     
      anglePID.setP(pidGainContainer.pGainAngle);
      anglePID.setI(pidGainContainer.iGainAngle);
      anglePID.setD(pidGainContainer.dGainAngle);
      anglePID.setIlimit(pidGainContainer.iLimitAngle);
    }
    
    timeMeasure = micros();
    elapsedMicros = timeMeasure - lastTimeMeasure;
    deltaT = (float)elapsedMicros * 0.000001;
    ledAddTime += deltaT;
    stablePositionTime += deltaT;
    lastTimeMeasure = timeMeasure;
    
    if(ledAddTime >= 1.0)
    {
      digitalWrite(ledPin, ledToggle);
      ledToggle = !ledToggle;
      ledAddTime = 0;
    }
    */
  }
}

void commTaskHandler(void * pvParameters)
{
  PidGainContainer pidContainer;
  Preferences preferences;
  BluetoothSerial btInstance;
  btInstance.begin("ESP32-BalancingRobot");
  
  Config &myConfig = Config::getConfig();
  myConfig.init(&preferences);

  pidContainer.pGainAngle = myConfig.getPGainAngle();
  pidContainer.iGainAngle = myConfig.getIGainAngle();
  pidContainer.dGainAngle = myConfig.getDGainAngle();
  pidContainer.iLimitAngle = myConfig.getILimitAngle();

  xQueueSend(messageQueue, &pidContainer.pGainAngle, portMAX_DELAY);
  xQueueSend(messageQueue, &pidContainer.iGainAngle, portMAX_DELAY);
  xQueueSend(messageQueue, &pidContainer.dGainAngle, portMAX_DELAY);
  xQueueSend(messageQueue, &pidContainer.iLimitAngle, portMAX_DELAY);

  Cmd cmd(&myConfig, &messageQueue);
  CommandContainer cmdContainer;
  char responseBuffer[PAYLOAD_LENGTH + COMMAND_LENGTH];
  uint32_t responseLenght = 0;
  
  for(;;)
  {
    if(btInstance.available())
    {
      if(cmd.requestHandler(btInstance.read(), &cmdContainer))
      {
        // TODO process request and transmit a response
        // C:\Users\xxxxx\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.2
        cmd.processRequest(&cmdContainer);
        cmd.buildResponse(&cmdContainer, responseBuffer);

        responseLenght = strlen(responseBuffer);
        responseBuffer[responseLenght++] = CR;
        btInstance.write((uint8_t*) responseBuffer, responseLenght);
      }
    }
    else if(Serial1.available() >= 9)
    {
      while(Serial1.available()>=9)
      {
        if((0x59 == Serial1.read()) && (0x59 == Serial1.read())) //Byte1 & Byte2
        {
          unsigned int t1 = Serial1.read(); //Byte3
          unsigned int t2 = Serial1.read(); //Byte4
      
          t2 <<= 8;
          t2 += t1;
          Serial.print(t2);
          Serial.print('\t');
      
          t1 = Serial1.read(); //Byte5
          t2 = Serial1.read(); //Byte6
      
          t2 <<= 8;
          t2 += t1;
          Serial.println(t2);
      
          for(int i=0; i<3; i++) 
          { 
              Serial1.read(); ////Byte7,8,9
          }
        }
      }
    }
    else
    {
      delay(10);
    }
  }
}

/* only runs with default i2c speed? */
void searchForDevices()
{  
  uint8_t cnt = 0;

  Wire.begin();
  Wire.setClock(ADAFRUIT_I2C_CLK);
  
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
