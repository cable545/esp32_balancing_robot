#include "config.h"

#define PREFERENCES_KEY "cg"
#define P_GAIN_ANGLE_KEY "pga"
#define I_GAIN_ANGLE_KEY "iga"
#define D_GAIN_ANGLE_KEY "dga"
#define I_LIMIT_ANGLE_KEY "ila"

const float P_GAIN_ANGLE_DEFAULT = 1.0;
const float I_GAIN_ANGLE_DEFAULT = 0.0;
const float D_GAIN_ANGLE_DEFAULT = 0.0;
const float I_LIMIT_ANGLE_DEFAULT = 200.0;

Config &Config::getConfig()
{
  static Config myConfig;

  return myConfig;
}

void Config::init(Preferences *p)
{
  p_prefs = p;
  loadDataFromFlash();
}

void Config::loadDataFromFlash()
{
  p_prefs->begin(PREFERENCES_KEY, true);
  pGainAngle = p_prefs->getFloat(P_GAIN_ANGLE_KEY, P_GAIN_ANGLE_DEFAULT);
  iGainAngle = p_prefs->getFloat(I_GAIN_ANGLE_KEY, I_GAIN_ANGLE_DEFAULT);
  dGainAngle = p_prefs->getFloat(D_GAIN_ANGLE_KEY, D_GAIN_ANGLE_DEFAULT);
  iLimitAngle = p_prefs->getFloat(I_LIMIT_ANGLE_KEY, I_LIMIT_ANGLE_DEFAULT);
  p_prefs->end();

  mirrorConfigData();
}

uint32_t Config::update()
{
  uint32_t storedBytes = 0;

  p_prefs->begin(PREFERENCES_KEY, false);

  if(pGainAngleTmp != pGainAngle)
  {
    storedBytes += p_prefs->putFloat(P_GAIN_ANGLE_KEY, pGainAngleTmp);
    pGainAngle = pGainAngleTmp;
  }
  else if(iGainAngleTmp != iGainAngle)
  {
    storedBytes += p_prefs->putFloat(I_GAIN_ANGLE_KEY, iGainAngleTmp);
    iGainAngle = iGainAngleTmp;
  }
  else if(dGainAngleTmp != dGainAngle)
  {
    storedBytes += p_prefs->putFloat(D_GAIN_ANGLE_KEY, dGainAngleTmp);
    dGainAngle = dGainAngleTmp;
  }
  else if(iLimitAngleTmp != iLimitAngle)
  {
    storedBytes += p_prefs->putFloat(I_LIMIT_ANGLE_KEY, iLimitAngleTmp);
    iLimitAngle = iLimitAngleTmp;
  }

  p_prefs->end();

  return storedBytes;
}

void Config::mirrorConfigData()
{
  pGainAngleTmp = pGainAngle;
  iGainAngleTmp = iGainAngle;
  dGainAngleTmp = dGainAngle;
  iLimitAngleTmp = iLimitAngle;
}
