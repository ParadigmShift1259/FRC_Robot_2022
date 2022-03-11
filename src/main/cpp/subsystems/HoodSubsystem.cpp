#include "subsystems/HoodSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <iostream>

using namespace HoodConstants;
using namespace std;
using namespace frc;

HoodSubsystem::HoodSubsystem() : m_servo(kPWMPort)
{
}

void HoodSubsystem::Periodic()
{
    SmartDashboard::PutNumber("Hood Servo Pos", m_servo.Get());
}

void HoodSubsystem::SetServoPosition(double position) 
{
    m_servo.Set(position);
}

void HoodSubsystem::SetByDistance(double distHubCenter)
{
    double distance = distHubCenter - kHubOffsetRimToCenter.to<double>();

    if (distance > 0)
    {
        m_calculation.CalcInitRPMs(meter_t(distance), kTargetDistIntoHub);
        degree_t initAngle = m_calculation.GetInitAngle();
        double x = initAngle.to<double>();
        double c = SmartDashboard::GetNumber("Hoodangle Constant", 0.0317);
        double hoodangle = 0.33 - c * x + 0.000816 * x * x;
        if (hoodangle != hoodangle)
        {
            hoodangle = 0.33 - 0.0317 * x + 0.000816 * x * x;
        }
        hoodangle = std::clamp(hoodangle, HoodConstants::kMin, HoodConstants::kMax);
        SetServoPosition(hoodangle);

        SmartDashboard::PutNumber("Init Angle: ", initAngle.to<double>());
        SmartDashboard::PutNumber("Hood Angle:", hoodangle);
        SmartDashboard::PutNumber("Hoodangle Constant", c);
   }
}