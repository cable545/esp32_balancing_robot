#include <Wire.h>
#include "AdafruitMpu.h"

#define TAU 0.085//0.0796

#define ACC_OFFSET_LOOPS 10000
#define GYRO_OFFSET_LOOPS 10000

extern float DELTA_T;

static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb

static float cal_matrix[3][3]={{ 1.044623, -0.011422, -0.004677},
                          {-0.011422,  1.055071, -0.012410},
                          {-0.004677, -0.012410,  1.154486}};
  
static float cal_offsets[3] = {-16.127077, 83.156128, 64.799910};

float x_gyro_offset = -2.218852, y_gyro_offset = -43.924377, z_gyro_offset = -139.007294;
float x_acc_offset = -0.006899, y_acc_offset = 0.008814, z_acc_offset = 0.023406;

AdafruitMpu::AdafruitMpu(void)
{
  eulerAngles.roll = 0;
  eulerAngles.pitch = 0;
  eulerAngles.yaw = 0;
}

bool AdafruitMpu::init(void)
{
  bool result = false;

  Wire.begin();
  Wire.setClock(ADAFRUIT_I2C_CLK);

  result = accInit();
  result = gyroInit();

  return result;
}

bool AdafruitMpu::accInit(void)
{
  // Xen, Yen, Zen, normal mode and 1344 Hz update rate -> 10010111
  accWriteReg(LSM303_REGISTER_ACC_CTRL_REG1_A, 0x97);

  // select full scale range to 8G -> 00100000
  accWriteReg(LSM303_REGISTER_ACC_CTRL_REG4_A, 0x20);

  // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
  // if we are connected or not
  uint8_t reg1_a = accReadReg(LSM303_REGISTER_ACC_CTRL_REG1_A);
  if (reg1_a != 0x97)
    return false;

  return true;
}

bool AdafruitMpu::gyroInit(void)
{
  gyroData.x = 0;
  gyroData.y = 0;
  gyroData.z = 0;

  gyroEulerData.roll = 0.0;
  gyroEulerData.pitch = 0.0;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = gyroReadReg(GYRO_REGISTER_WHO_AM_I);
  //Serial.println(id, HEX);
  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
    return false;

  /*
   *  enable only X axis, power mode -> normal,
   *  output data rate -> 800 Hz without cut off frequency
   *  => 11101010
   */ 
  gyroWriteReg(GYRO_REGISTER_CTRL_REG1, 0xEA);

  // Full scale selection to 500 dps
  gyroWriteReg(GYRO_REGISTER_CTRL_REG4, 0x10);
  
  return true;
}

uint8_t AdafruitMpu::accReadReg(uint8_t regAddress)
{
  uint8_t value;

  Wire.beginTransmission(LSM303_ADDRESS_ACCEL);
  Wire.write((uint8_t)regAddress);
  Wire.endTransmission();
  Wire.requestFrom(LSM303_ADDRESS_ACCEL, (uint8_t)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void AdafruitMpu::accWriteReg(uint8_t regAddress, uint8_t value)
{
  Wire.beginTransmission(LSM303_ADDRESS_ACCEL);
  Wire.write((uint8_t)regAddress);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

void AdafruitMpu::accRead(void)
{
  int16_t accX, accY, accZ;
  
  // Read the accelerometer
  Wire.beginTransmission((uint8_t)LSM303_ADDRESS_ACCEL);
  Wire.write(LSM303_REGISTER_ACC_OUT_X_L_A | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)LSM303_ADDRESS_ACCEL, (uint8_t)6);

  // Wait around until enough data is available
 // while (Wire.available() < 6);
  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  accX = (int16_t)(xlo | (xhi << 8)) >> 2;
  accY = (int16_t)(ylo | (yhi << 8)) >> 2;
  accZ = (int16_t)(zlo | (zhi << 8)) >> 2;

  accData.x = (float)(accX * _lsm303Accel_MG_LSB);
  accData.y = (float)(accY * _lsm303Accel_MG_LSB);
  accData.z = (float)(accZ * _lsm303Accel_MG_LSB);
}

void AdafruitMpu::accCalcAngles(void)
{
  accEulerData.roll = ((atan2f(accData.y, accData.z)) * RAD_DEG);
  
  if(accEulerData.roll > 90)
    accEulerData.pitch = ((atan2f(accData.x, accData.y)) * RAD_DEG);
  else if(accEulerData.roll < -90)
    accEulerData.pitch = ((atan2f(accData.x, -accData.z)) * RAD_DEG);
  else 
    accEulerData.pitch = ((atan2f(accData.x, accData.z)) * RAD_DEG);
}

uint8_t AdafruitMpu::gyroReadReg(uint8_t regAddress)
{
  uint8_t value;

  Wire.beginTransmission((uint8_t)L3GD20H_GYRO_ADDRESS);
  Wire.write((uint8_t)regAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)L3GD20H_GYRO_ADDRESS, (uint8_t)1);
  value = Wire.read();

  return value;
}

void AdafruitMpu::gyroWriteReg(uint8_t regAddress, uint8_t value)
{
  Wire.beginTransmission(L3GD20H_GYRO_ADDRESS);
  Wire.write((uint8_t)regAddress);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

void AdafruitMpu::gyroRead(void)
{
  uint8_t xhi, xlo, ylo, yhi, zlo, zhi;

  Wire.beginTransmission(L3GD20H_GYRO_ADDRESS);
  // Make sure to set address auto-increment bit
  Wire.write(GYRO_REGISTER_OUT_X_L | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(L3GD20H_GYRO_ADDRESS, (uint8_t)6);

  xlo = Wire.read();
  xhi = Wire.read();
  ylo = Wire.read();
  yhi = Wire.read();
  zlo = Wire.read();
  zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  gyroData.x = (int16_t)(xlo | (xhi << 8));
  gyroData.y = (int16_t)(ylo | (yhi << 8));
  gyroData.z = (int16_t)(zlo | (zhi << 8));

  // Compensate values depending on the resolution
  gyroData.x *= GYRO_SENSITIVITY_500DPS;
  gyroData.y *= GYRO_SENSITIVITY_500DPS;
  gyroData.z *= GYRO_SENSITIVITY_500DPS;
}

void AdafruitMpu::gyroReadX(void)
{
  int16_t gyroX;

  Wire.beginTransmission(L3GD20H_GYRO_ADDRESS);
  // Make sure to set address auto-increment bit
  Wire.write(GYRO_REGISTER_OUT_X_L | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(L3GD20H_GYRO_ADDRESS, (uint8_t)2);

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  gyroX = (int16_t)(xlo | (xhi << 8));

  // Compensate values depending on the resolution
  gyroData.x = (float)(gyroX * GYRO_SENSITIVITY_500DPS);
}

void AdafruitMpu::gyroCalcAngles(float deltaT)
{
  gyroEulerData.roll += (gyroData.x * deltaT);
  gyroEulerData.pitch += (gyroData.y * deltaT);
  gyroEulerData.yaw += (gyroData.z * deltaT);
}

float AdafruitMpu::processingFilter(float angularRate, float accAngle, float angle, float deltaT)
{
  float a = TAU / (TAU + deltaT);
  
  return a * (angle + ((angularRate) * deltaT)) + ((1.0 - a) * (accAngle));
}

float AdafruitMpu::calculatePitchAngle(float deltaT)
{
  eulerAngles.pitch = processingFilter(gyroData.y, accEulerData.pitch, eulerAngles.pitch, deltaT);
  return eulerAngles.pitch;
}

float AdafruitMpu::calculateRollAngle(float deltaT)
{
  eulerAngles.roll = processingFilter(gyroData.x, accEulerData.roll, eulerAngles.roll, deltaT);
  return eulerAngles.roll;
}

void AdafruitMpu::accOffsetCalc(ImuData_t* accOffsets)
{
  accOffsets->x = 0;
  accOffsets->y = 0;
  accOffsets->z = 0;
  
  for(uint32_t index = 0; index < ACC_OFFSET_LOOPS; index++)
  {
    accRead();
    accOffsets->x += accData.x;
    accOffsets->y += accData.y;
    accOffsets->z += (accData.z - 1.0);    //z-value minus 1g
  }
  
  accOffsets->x /= ACC_OFFSET_LOOPS;
  accOffsets->y /= ACC_OFFSET_LOOPS;
  accOffsets->z /= ACC_OFFSET_LOOPS;
}

void AdafruitMpu::accReadComp(void)
{
  accRead();
  accData.x -= x_acc_offset;
  accData.y -= y_acc_offset;
  accData.z -= z_acc_offset;
}

void AdafruitMpu::gyroOffsetCalc(ImuData_t* gyroOffsets)
{
  gyroOffsets->x = 0;
  gyroOffsets->y = 0;
  gyroOffsets->z = 0;
  
  for(uint32_t index = 0; index < GYRO_OFFSET_LOOPS; index++)
  {
    gyroRead();
    gyroOffsets->x += gyroData.x;
    gyroOffsets->y += gyroData.y;
    gyroOffsets->z += gyroData.z;
  }
  
  gyroOffsets->x /= GYRO_OFFSET_LOOPS;
  gyroOffsets->y /= GYRO_OFFSET_LOOPS;
  gyroOffsets->z /= GYRO_OFFSET_LOOPS;
}

void AdafruitMpu::gyroXOffsetCalc(ImuData_t* gyroOffsets)
{
  gyroOffsets->x = 0;
  
  for(uint32_t index = 0; index < GYRO_OFFSET_LOOPS; index++)
  {
    gyroReadX();
    gyroOffsets->x += gyroData.x;
  }
  
  gyroOffsets->x /= GYRO_OFFSET_LOOPS;
}

void AdafruitMpu::gyroReadComp(void)
{
  gyroRead();
  gyroData.x -= x_gyro_offset;
  gyroData.y -= y_gyro_offset;
  gyroData.z -= z_gyro_offset;
}

void AdafruitMpu::gyroReadXComp(void)
{
  gyroReadX();
  gyroData.x -= x_gyro_offset;
}

void AdafruitMpu::executeCalibration(ImuData_t* gyroOffsets, ImuData_t* accOffsets)
{
  digitalWrite(13, HIGH);
  
  gyroXOffsetCalc(gyroOffsets);
  accOffsetCalc(accOffsets);
  
  digitalWrite(13, LOW);
}

///*-----------------------------------------------------------------------------------------------------*/
///*                                  functions for the adafruit modul                                   */
///*-----------------------------------------------------------------------------------------------------*/
//
///*
// *  a wrapper function for a "complete" reading from the adafruit modul
// */
//void read_sensor_data(void)
//{ 
//  if(lsm303_acc_read_comp(&acc_values) == TRUE)
//  {
//    acc_low_pass_filter(&acc_values);
//  }
////  else
////    printf("Err acc read\r\n");
//  
//  if(l3gd_gyro_read_comp(&gyro_values) == TRUE)
//  {
//    gyro_low_pass_filter(&gyro_values);
//  }
////  else
////    printf("Err gyr read\r\n");
//}
//

//
//
//
//
