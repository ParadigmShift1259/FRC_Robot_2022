#include "Gyro.h"


Gyro2::Gyro2() :
    m_gyro(0)
{

}

double Gyro2::GetHeading()
{
    auto retVal = std::remainder(m_gyro.GetFusedHeading(), 360.0) * (kGyroReversed ? -1. : 1.);
    if (retVal > 180.0)
        retVal -= 360.0;

    return retVal;
}

void Gyro2::ZeroHeading()
{
    m_gyro.SetFusedHeading(0.0, 0);
}

double Gyro2::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates);
    return turnRates[2] * (kGyroReversed ? -1. : 1.); 
}