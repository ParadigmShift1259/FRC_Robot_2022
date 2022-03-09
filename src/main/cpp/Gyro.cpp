#include "Gyro.h"

namespace Team1259
{

Gyro::Gyro() :
    m_gyro(0)
{

}

double Gyro::GetHeading()
{
    auto retVal = std::remainder(m_gyro.GetFusedHeading(), 360.0) * (kGyroReversed ? -1. : 1.);
    if (retVal > 180.0)
        retVal -= 360.0;

    return retVal;
}

void Gyro::ZeroHeading()
{
    m_gyro.SetFusedHeading(0.0, 30);
}

void Gyro::SetHeading(double heading)
{
int err;

printf("initial gyro: %f  ", GetHeading());
for (int n=0; n<100; n++)
    err = m_gyro.SetFusedHeading(heading * (kGyroReversed ? -1. : 1.), 30);
printf("heading to set: %f  ", heading);  
printf("returned error code: %d  ", err); // Observed to return 0 yet still not set heading!
printf("finial gyro: %f\n", GetHeading());
}

double Gyro::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates);
    return turnRates[2] * (kGyroReversed ? -1. : 1.); 
}

}   //namespace Team1259
