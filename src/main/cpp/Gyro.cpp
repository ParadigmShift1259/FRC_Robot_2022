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
    SetHeading(0.0);
}

void Gyro::SetHeading(double heading)
{
int err;

// printf("initial gyro: %f  ", GetHeading());
// printf("heading to set: %f  ", heading);  
for (int n=0; n<100; n++)
    {
    err = m_gyro.SetFusedHeading(heading * (kGyroReversed ? -1. : 1.), 30);
    if (err)
        printf("SetFusedHeading() failed with error code: %d  ", err); 
    else if(fabs(GetHeading() - heading) < 0.1)  // *** Check needed becuase SetFusedHeading() observed to faile yet still return 0! ***
        {
        // printf("finial gyro: %f\n", GetHeading())
        return;
        }
    }
    printf("************************* FAILED TO SET GYRO HEADING! **********************************");
}

double Gyro::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates);
    return turnRates[2] * (kGyroReversed ? -1. : 1.); 
}

}   //namespace Team1259
