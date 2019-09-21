#ifndef _MY_STEPPER_H
#define _MY_STEPPER_H

#define STEP_PIN_LEFT_MOT 32
#define DIR_PIN_LEFT_MOT 15
#define STEP_PIN_RIGHT_MOT 33
#define DIR_PIN_RIGHT_MOT 27

#define LEFT_STEPPER_TIMER_0 0
#define RIGHT_STEPPER_TIMER_1 1

class MyStepper
{
  public:
    void step();
    void stepperTimerStart();
    void setTimerValue(uint64_t timerValue);
  
    static MyStepper& getRightStepper();
    static MyStepper& getLeftStepper();
  private:
    MyStepper(uint8_t sp, uint8_t dp, uint8_t tnr, void (*timerIntHandler)(void));
    void initTimer(void (*timerIntHandler)(void));
  
    //static void IRAM_ATTR rightMotTimerIntHandler();
    //static void IRAM_ATTR leftMotTimerIntHandler();
  
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin; // TODO define enable pin to be able to enable and disable the stepper
    uint8_t timerNr;
  
    hw_timer_t* hwTimer;
};

#endif
