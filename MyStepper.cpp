#include "Arduino.h"
#include "MyStepper.h"

void IRAM_ATTR rightMotTimerIntHandler()
{
  MyStepper::getRightStepper().step();
}

void IRAM_ATTR leftMotTimerIntHandler()
{
  MyStepper::getLeftStepper().step();
}

MyStepper& MyStepper::getRightStepper()
{
  static MyStepper rightStepper = MyStepper(
    STEP_PIN_RIGHT_MOT,
    DIR_PIN_RIGHT_MOT,
    RIGHT_STEPPER_TIMER_1,
    RIGHT_STEPPER_FORWARD_DIR_LEVEL,
    RIGHT_MOTOR_ID,
    &rightMotTimerIntHandler
  );

  return rightStepper;
}

MyStepper& MyStepper::getLeftStepper()
{
  static MyStepper leftStepper = MyStepper(
    STEP_PIN_LEFT_MOT,
    DIR_PIN_LEFT_MOT,
    LEFT_STEPPER_TIMER_0,
    LEFT_STEPPER_FORWARD_DIR_LEVEL,
    LEFT_MOTOR_ID,
    &leftMotTimerIntHandler
  );

  return leftStepper;
}

MyStepper::MyStepper(uint8_t sp, uint8_t dp, uint8_t tnr, uint8_t forwardDirLevel, uint8_t mId, void (*timerIntHandler)(void))
{
  stepPin = sp;
  dirPin = dp;
  timerNr = tnr;
  forwardDirectionLevel = forwardDirLevel;
  backwardDirectionLevel = !forwardDirLevel;
  motorID = mId;

  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);

  initTimer(timerIntHandler);
}

void MyStepper::step()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(stepPin, LOW);
}

void MyStepper::setDirection(uint8_t dir)
{
  digitalWrite(dirPin, dir);
}

void MyStepper::initTimer(void (*timerIntHandler)(void))
{
  hwTimer = timerBegin(timerNr, 40, true);        // prescaler 40 set timer frequency to 2Mhz, counts up
  timerAttachInterrupt(hwTimer, timerIntHandler, true);
  timerAlarmWrite(hwTimer, 2000000, true);       // alarmvalue and autoreload
  timerAlarmEnable(hwTimer);
  timerStop(hwTimer);
}

void MyStepper::stepperTimerStart()
{
  timerStart(hwTimer);
}

void MyStepper::setTimerValue(uint64_t timerValue)
{
  timerAlarmWrite(hwTimer, timerValue, true);       // alarmvalue and autoreload
}

void MyStepper::setMotorSpeed(float motorSpeed)
{
  long timerPeriod;

  timerAlarmDisable(hwTimer);
  
  if(motorSpeed > 0)
  {
    timerPeriod = 2000000 / motorSpeed; // 2Mhz timer
    currentDirection = backwardDirectionLevel;
    setDirection(currentDirection);
  }
  else if(motorSpeed < 0)
  {
    timerPeriod = 2000000 / -motorSpeed;
    currentDirection = forwardDirectionLevel;
    setDirection(currentDirection);
  }
  else
  {
    timerPeriod = 512000;
    currentDirection = NO_DIRECTION;
  }

  if (timerPeriod > 512000)   // Check for maximun period without overflow
    timerPeriod = 512000;
  else if(timerPeriod < MIN_TIMER_PERIOD)
    timerPeriod = MIN_TIMER_PERIOD;

  Serial1.print(motorID); Serial1.print(" ");
  Serial1.print(motorSpeed); Serial1.print(" ");
  Serial1.println(timerPeriod);// Serial1.print(" ");
  //Serial1.print(currentDirection);
 // Serial1.println();

  setTimerValue(timerPeriod);
  timerAlarmEnable(hwTimer);
}
