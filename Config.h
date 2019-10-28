#ifndef _CONFIG_H
#define _CONFIG_H

#include <Preferences.h>

class Config
{
  public:
    static Config& getConfig();

    void init(Preferences *p);
    void end() {p_prefs->end();}

    float getPGainAngle() {return pGainAngleTmp;}
    float getIGainAngle() {return iGainAngleTmp;}
    float getDGainAngle() {return dGainAngleTmp;}
    float getILimitAngle() {return iLimitAngleTmp;}

    void setPGainAngle(float value) {pGainAngleTmp = value;}
    void setIGainAngle(float value) {iGainAngleTmp = value;}
    void setDGainAngle(float value) {dGainAngleTmp = value;}
    void setILimitAngle(float value) {iLimitAngleTmp = value;}
    
    uint32_t update();

  private:
    Config() {}
    
    void loadDataFromFlash();
    void mirrorConfigData();

    float pGainAngle;
    float iGainAngle;
    float dGainAngle;
    float iLimitAngle;

    float pGainAngleTmp;
    float iGainAngleTmp;
    float dGainAngleTmp;
    float iLimitAngleTmp;

    Preferences *p_prefs;
};

#endif
