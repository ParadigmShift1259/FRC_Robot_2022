#include "common/Gyro.h"


Gyro::Gyro() : m_gyro(0) {}

double Gyro::GetHeading()
{
    auto heading = std::remainder(m_gyro.GetFusedHeading(), 360.0) * (kGyroReversed ? -1. : 1.);
    if (heading > 180.0)
        heading -= 360.0;

    return heading;
}

void Gyro::ZeroHeading()
{
    m_gyro.SetFusedHeading(0.0, 0);
}

double Gyro::GetTurnRate()
{
    double turnRates [3] = {0, 0, 0};
    m_gyro.GetRawGyro(turnRates);
    return turnRates[2] * (kGyroReversed ? -1. : 1.); 
}