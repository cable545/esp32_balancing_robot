#ifndef _PID_H
#define _PID_H

typedef struct
{
  float pGainAngle;
  float iGainAngle;
  float dGainAngle;
  float iLimitAngle;
} PidGainContainer;

class Pid
{
  public:
    Pid(float p, float i, float d, float iLimit);
    float updatePID(float target, float current, float dt);
    void resetPID();
    void setGains(float p, float i, float d);
    
    void setP(float p) {P = p;}
    void setI(float i) {I = i;}
    void setD(float d) {D = d;}
    void setIlimit(float limit) {I_limit = limit;}
  
    float getP() {return P;}
    float getI() {return I;}
    float getD() {return D;}
    float getILimit() {return I_limit;}
  
  private: 
    float P; 
    float I; 
    float D; 
    float I_limit; 
    float integratedError; 
    float lastError; 
};

#endif
