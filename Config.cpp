#include "config.h"

#define PREFERENCES_KEY "cg"
#define P_GAIN_ANGLE_KEY "pga"
#define I_GAIN_ANGLE_KEY "iga"
#define I_LIMIT_ANGLE_KEY "ila"

const float P_GAIN_ANGLE_DEFAULT = 50.0;
const float I_GAIN_ANGLE_DEFAULT = 2.5;
const float I_LIMIT_ANGLE_DEFAULT = 200.0;

Config& Config::getConfig()
{
  static Config myConfig = Config();

  return myConfig;
}

Config::Config()
{
  prefs.begin(PREFERENCES_KEY, true);
  loadDataFromFlash();
}

Config::~Config()
{
  prefs.end();
}

void Config::loadDataFromFlash()
{
  pGainAngle = prefs.getFloat(P_GAIN_ANGLE_KEY, P_GAIN_ANGLE_DEFAULT);
  iGainAngle = prefs.getFloat(I_GAIN_ANGLE_KEY, I_GAIN_ANGLE_DEFAULT);
  iLimitAngle = prefs.getFloat(I_LIMIT_ANGLE_KEY, I_LIMIT_ANGLE_DEFAULT);

  mirrorConfigData();
}

bool Config::update()
{
  bool updated = true;

  if(pGainAngleTmp != pGainAngle)
  {
    prefs.putFloat(P_GAIN_ANGLE_KEY, pGainAngleTmp);
    pGainAngle = pGainAngleTmp;
  }
  else if(iGainAngleTmp != iGainAngle)
  {
    prefs.putFloat(I_GAIN_ANGLE_KEY, iGainAngleTmp);
    iGainAngle = iGainAngleTmp;
  }
  else if(iLimitAngleTmp != iLimitAngle)
  {
    prefs.putFloat(I_LIMIT_ANGLE_KEY, iLimitAngleTmp);
    iLimitAngle = iLimitAngleTmp;
  }
  else
  {
    updated = false;
  }

  return updated;
}

void Config::mirrorConfigData()
{
  pGainAngleTmp = pGainAngle;
  iGainAngleTmp = iGainAngle;
  iLimitAngleTmp = iLimitAngle;
}
