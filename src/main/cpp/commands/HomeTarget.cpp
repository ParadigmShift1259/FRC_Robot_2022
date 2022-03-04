#include "commands/HomeTarget.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

units::meter_t kHubOffsetRimToCenter = units::foot_t(2.0);

HomeTarget::HomeTarget(   FlywheelSubsystem *flywheel
                        , TurretSubsystem *turret
                        , HoodSubsystem *hood
                        , VisionSubsystem &vision
                        , bool *turretready
                        , bool *firing
                        , bool *finished
                        , GetYvelocityCallback yVelocityCb)
    : m_flywheel(flywheel)
    , m_turret(turret)
    , m_hood(hood)
    , m_turretready(turretready)
    , m_vision(vision)
    , m_firing(firing)
    , m_finished(finished)
    , m_yVelocityCb(yVelocityCb)
{
    AddRequirements({flywheel, turret, hood});
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void HomeTarget::Initialize()
{
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void HomeTarget::Execute()
{
    // Homes flywheel, turret, and hood to the right angles through a formula
    SmartDashboard::PutBoolean("TEST_VIS_ACTIVE", m_vision.GetValidTarget());

    if (!m_vision.GetValidTarget())
        return;

    double distance = m_vision.GetHubDistance(true) - kHubOffsetRimToCenter.to<double>();

    // if (std::isnan(distance))
    if (distance != distance)
    {
        return;
    }

    // const std::map<double, double> distCompensation =
    //     {
    //         std::make_pair(6.0 * 12.0, 1100.0), std::make_pair(8.0 * 12.0, 1200.0), std::make_pair(10.0 * 12.0, 1400.0), std::make_pair(12.0 * 12.0, 1650.0), std::make_pair(14.0 * 12.0, 1750.0), std::make_pair(16.0 * 12.0, 1900.0), std::make_pair(18.0 * 12.0, 2100.0), std::make_pair(20.0 * 12.0, 2100.0)};
    // auto it = distCompensation.lower_bound(distance / 12.0);

    double offset = 100.0 * distance / 12.0;

    // bool bUseLut = SmartDashboard::GetBoolean("UseLut", true);
    // if (bUseLut && it != distCompensation.end())
    // {
    //     offset = it->second;
    // }

    bool bUseFudgeFactor = SmartDashboard::GetBoolean("UseFudgeFactor", true);
    double fudge = 0.0;
    if (bUseFudgeFactor)
    {
        fudge = SmartDashboard::GetNumber("fudge", 100.0);
        offset += fudge;
    }

    // Try to hit close in shots
    if (distance <= 2.5)
    {
        offset -= 330;
    }

    revolutions_per_minute_t flywheelspeedInPRM = revolutions_per_minute_t(offset) + m_calculation.CalcInitRPMs(meter_t(distance), kHubOffsetRimToCenter);
    double flywheelspeed = flywheelspeedInPRM.to<double>();

    // Servo Pos    Measured Angle  Complement
    // 0.0	        50 deg          90 - 50 = 40
    // 0.4	        40 deg          90 - 40 = 50
    degree_t initAngle = m_calculation.GetInitAngle();
    // double hoodangle = (initAngle.to<double>() - 40.0) * 0.04;
    double x = initAngle.to<double>();
    // double hoodangle = 0.33 - 0.0317 * x + 0.000816 * x * x;
    // double c = SmartDashboard::GetNumber("Hoodangle Constant", 0.33);
    double c = SmartDashboard::GetNumber("Hoodangle Constant", 0.0317);
    double hoodangle = 0.33 - c * x + 0.000816 * x * x;
    if (hoodangle != hoodangle)
    {
        hoodangle = 0.33 - 0.0317 * x + 0.000816 * x * x;
    }
    hoodangle = std::clamp(hoodangle, HoodConstants::kMin, HoodConstants::kMax);
    m_hood->Set(hoodangle);

    // std::cout << "Init Angle: "<< initAngle.to<double>() << std::endl;
    // std::cout << "Hood servo set: "<< hoodangle << std::endl;
    // std::cout << "Distance: "<< distance << std::endl;
    // std::cout << "Flywheel RPM "<< flywheelspeed << std::endl;
    SmartDashboard::PutNumber("Init Angle: ", initAngle.to<double>());
    //SmartDashboard::PutNumber("Hood servo set: ", hoodangle);
    //SmartDashboard::PutNumber("Distance: ", distance);
    //SmartDashboard::PutNumber("Flywheel RPM ", flywheelspeed);
    //SmartDashboard::PutNumber("Hub angle ", m_vision.GetHubAngle());

    SmartDashboard::PutNumber("Hoodangle Constant", c);

    m_flywheel->SetRPM(flywheelspeed);

    SmartDashboard::PutBoolean("D_FIRE_AT_RPM", m_flywheel->IsAtRPM());
    //SmartDashboard::PutBoolean("D_FIRE_AT_SET", m_turret->isAtSetpoint());
    SmartDashboard::PutNumber("Hood Angle:", hoodangle);

    if (m_flywheel->IsAtRPM() && m_yVelocityCb() <= 1.1 * kSlowDriveSpeed.to<double>())
    {
        *m_turretready = true;
    }

    frc::SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
}

bool HomeTarget::IsFinished()
{
    return *m_turretready;
}

void HomeTarget::End(bool interrupted)
{
    
    //*m_finished = false;
    // m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
    // m_hood->Set(HoodConstants::kMax);
    // m_turret->TurnTo(TurretConstants::kStartingPositionDegrees);
}