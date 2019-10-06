#ifndef _CONFIG_H
#define _CONFIG_H

#include <Preferences.h>

class Config
{
  public:
    static Config& getConfig();

    float getPGainAngle() {return pGainAngleTmp;}
    float getIGainAngle() {return iGainAngleTmp;}
    float getILimitAngle() {return iLimitAngleTmp;}

    void setPGainAngle(float value) {pGainAngleTmp = value;}
    void setIGainAngle(float value) {iGainAngleTmp = value;}
    void setILimitAngle(float value) {iLimitAngleTmp = value;}

    bool update();

  private:
    Config();
    ~Config();
    void loadDataFromFlash();
    void mirrorConfigData();

    float pGainAngle;
    float iGainAngle;
    float iLimitAngle;

    float pGainAngleTmp;
    float iGainAngleTmp;
    float iLimitAngleTmp;

    Preferences prefs;
};

#endif
