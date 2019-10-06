#ifndef _MY_STEPPER_H
#define _MY_STEPPER_H

#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2

#define STEP_PIN_LEFT_MOT 32
#define DIR_PIN_LEFT_MOT 15
#define STEP_PIN_RIGHT_MOT 33
#define DIR_PIN_RIGHT_MOT 27

#define LEFT_STEPPER_TIMER_0 0
#define RIGHT_STEPPER_TIMER_1 1

#define RIGHT_STEPPER_FORWARD_DIR_LEVEL 0
#define LEFT_STEPPER_FORWARD_DIR_LEVEL 1
#define NO_DIRECTION -1

#define MIN_TIMER_PERIOD 150

class MyStepper
{
  public:
    void step();
    void setDirection(uint8_t dir);
    void stepperTimerStart();
    void setTimerValue(uint64_t timerValue);
    void setMotorSpeed(float motorSpeed);
    void enableInterrupt();
    void disableInterrupt();
  
    static MyStepper& getRightStepper();
    static MyStepper& getLeftStepper();
  private:
    MyStepper(uint8_t sp, uint8_t dp, uint8_t tnr, uint8_t forwardDirLevel, uint8_t mId, void (*timerIntHandler)(void));
    void initTimer(void (*timerIntHandler)(void));

    uint8_t motorID;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin; // TODO define enable pin to be able to enable and disable the stepper
    uint8_t timerNr;
    int8_t currentDirection;

    uint8_t forwardDirectionLevel;
    uint8_t backwardDirectionLevel;
  
    hw_timer_t* hwTimer;
};

#endif
