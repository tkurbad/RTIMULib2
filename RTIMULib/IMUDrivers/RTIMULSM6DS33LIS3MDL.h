////////////////////////////////////////////////////////////////////////////
//


#ifndef _RTIMULSM6DS33LIS3MDL_H
#define	_RTIMULSM6DS33LIS3MDL_H

#include "RTIMU.h"

class RTIMULSM6DS33LIS3MDL : public RTIMU
{
public:
    RTIMULSM6DS33LIS3MDL(RTIMUSettings *settings);
    ~RTIMULSM6DS33LIS3MDL();

    virtual const char *IMUName() { return "LSM6DS33 + LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM6DS33LIS3MDL; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroCTRL2();
    bool setGyroCTRL7();
    bool setAccelCTRL1();
    bool setCompassCTRL1();
    bool setCompassCTRL2();
    bool setCompassCTRL3();
    bool setCompassCTRL4();

    unsigned char m_accelGyroSlaveAddr;                     // I2C address of LSM6DS33 accel and gyro
    unsigned char m_compassSlaveAddr;                       // I2C address of LIS3MDL mag

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

#endif // _RTIMULSM6DS33LIS3MDL_H
