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

double HoodSubsystem::GetServoPosition() 
{
    m_servo.Get();
}

void HoodSubsystem::SetByDistance(double distHubCenter)
{
    if (m_bTestOverride)
    {
        printf("Hood servo test override supressing automatic hood adjustment\n");
        return;
    }

    double distance = distHubCenter - kHubOffsetRimToCenter.to<double>();

    if (distance != distance)
    {
        printf("distance is non a number\n");
        return;
    }

    if (distance > 0.0)
    {
        foot_t heightAboveHub = meter_t(distance).convert<foot>() * slope + foot_t(offset);
        SmartDashboard::PutNumber("HAH dist", distance * 39.37008);
        SmartDashboard::PutNumber("HAH", heightAboveHub.to<double>());
        m_flywheelSpeed = m_calculation.CalcInitRPMs(meter_t(distance), kTargetDistIntoHub, heightAboveHub).to<double>();
        SmartDashboard::PutNumber("FW Speed", m_flywheelSpeed);
        degree_t initAngle = m_calculation.GetInitAngle();
        double x = initAngle.to<double>();
        if (x != x)
        {
            printf("init angle is non a number\n");
            return;
        }

        //double c = SmartDashboard::GetNumber("Hoodangle Constant", 0.159);
        double hoodangle = -2.58 + 0.159 * x + -0.00298 * x * x + 0.0000216 * x * x * x;
        if (hoodangle != hoodangle)
        {
            hoodangle = -2.58 + 0.159 * x + -0.00298 * x * x + 0.0000216 * x * x * x;
        }
        hoodangle = std::clamp(hoodangle, HoodConstants::kMin, HoodConstants::kMax);
        if (hoodangle != hoodangle)
        {
            printf("hoodangle is non a number\n");
        }
        else
        {
            SetServoPosition(hoodangle);
        }

        SmartDashboard::PutNumber("Init Angle: ", initAngle.to<double>());
        SmartDashboard::PutNumber("Hood Angle:", hoodangle);
        //SmartDashboard::PutNumber("Hoodangle Constant", c);
   }
}