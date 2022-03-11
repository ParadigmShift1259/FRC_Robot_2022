#include "commands/HomeTarget.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

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
    , m_vision(vision)
    , m_turretready(turretready)
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
#ifdef COMMAND_TIMING
    m_startTime = m_timer.GetFPGATimestamp().to<double>();
    printf("timestamp start home target %.3f\n", m_startTime);
#endif
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

    double distToHubCenter = m_vision.GetHubDistance(true);
    double distance = distToHubCenter - kHubOffsetRimToCenter.to<double>();

    // if (std::isnan(distance))
    if (distance != distance)
    {
        return;
    }

    double offset = 0.0;
    double flywheelspeed = offset + m_calculation.CalcInitRPMs(meter_t(distance), kTargetDistIntoHub).to<double>();
    m_flywheel->SetRPM(flywheelspeed);

    m_hood->SetByDistance(distToHubCenter);

    SmartDashboard::PutBoolean("D_FIRE_AT_RPM", m_flywheel->IsAtRPM());
    SmartDashboard::PutBoolean("D_FIRE_AT_SET", m_turret->isAtSetpoint());

    //if (m_flywheel->IsAtRPM() m_yVelocityCb() <= 1.1 * kSlowDriveSpeed.to<double>())
    if (m_flywheel->IsAtRPM() && m_turret->isAtSetpoint() && m_yVelocityCb() <= 1.1 * kSlowDriveSpeed.to<double>())
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
#ifdef COMMAND_TIMING
    auto endTime = m_timer.GetFPGATimestamp().to<double>();
    auto elapsed = endTime - m_startTime;
    printf("timestamp end home target %.3f elapsed %.3f\n", endTime, elapsed);
#endif
}  
