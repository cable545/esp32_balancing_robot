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
  static MyStepper rightStepper = MyStepper(STEP_PIN_RIGHT_MOT, DIR_PIN_RIGHT_MOT, RIGHT_STEPPER_TIMER_1, &rightMotTimerIntHandler);

  return rightStepper;
}

MyStepper& MyStepper::getLeftStepper()
{
  static MyStepper leftStepper = MyStepper(STEP_PIN_LEFT_MOT, DIR_PIN_LEFT_MOT, LEFT_STEPPER_TIMER_0, &leftMotTimerIntHandler);

  return leftStepper;
}

MyStepper::MyStepper(uint8_t sp, uint8_t dp, uint8_t tnr, void (*timerIntHandler)(void))
{
  stepPin = sp;
  dirPin = dp;
  timerNr = tnr;

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
