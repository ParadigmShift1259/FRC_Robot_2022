#include "commands/HomeTarget.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

HomeTarget::HomeTarget(   frc::XboxController* controller
                        , FlywheelSubsystem* flywheel
                        , TurretSubsystem* turret
                        , HoodSubsystem* hood
                        , VisionSubsystem& vision
                        , bool* turretready
                        , bool* firing
                        , bool* finished)
    : m_controller(controller)
    , m_flywheel(flywheel)
    , m_turret(turret)
    , m_hood(hood)
    , m_turretready(turretready)
    , m_vision(vision)
    , m_firing(firing)
    , m_finished(finished)
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

    double distance = m_vision.GetHubDistance(true);
    //double flywheelspeed = 1687.747 + 15.8111 * distance - 0.058079 * pow(distance, 2) + 0.00008892342 * pow(distance, 3);
    double flywheelspeed = m_calculation.GetInitRPMS(meter_t(distance)).to<double>() / FlywheelConstants::kGearRatio;

    // Quintic regression calculated 3/27
    // https://mycurvefit.com/
    //y=11.20831-0.2645223*x+0.002584349*x^{2}-0.00001250923*x^{3}+2.986403\cdot10^{-8}*x^{4}-2.81104\cdot10^{-11}*x^{5}
    //double hoodangle = 11.20831 - 0.2645223 * distance + 0.002584349 * pow(distance, 2) - 0.00001250923 * pow(distance, 3) + 2.986403E-8 * pow(distance, 4) - 2.81104E-11 * pow(distance, 5);
    
    // Servo Pos    Measured Angle  Complement
    // 0.0	        50 deg          90 - 50 = 40
    // 0.4	        40 deg          90 - 40 = 50
    degree_t initAngle = m_calculation.GetInitAngle();
    double hoodangle = (initAngle.to<double>() - 40.0) * 0.04;
    hoodangle = std::clamp(hoodangle, HoodConstants::kMin, HoodConstants::kMax);

    std::cout << "Init Angle: "<< initAngle.to<double>() << std::endl;
    std::cout << "Hood servo set: "<< hoodangle << std::endl;
    std::cout << "Distancet: "<< distance << std::endl;
    std::cout << "Flywheel RPM "<< flywheelspeed << std::endl;
    SmartDashboard::PutNumber("Init Angle: ", initAngle.to<double>());
    SmartDashboard::PutNumber("Hood servo set: ", hoodangle);
    SmartDashboard::PutNumber("Distancet: ", distance);
    SmartDashboard::PutNumber("Flywheel RPM ", flywheelspeed);

    double angleOverride = 0;
    // double turretXRot = m_controller->GetY(frc::GenericHID::kRightHand) * -1.0;
    // double turretYRot = m_controller->GetX(frc::GenericHID::kRightHand);

    //double turretXRot = m_controller->GetRightY() * -1.0;
    //double turretYRot = m_controller->GetRightX();

    // if (m_controller->GetBumperPressed(GenericHID::JoystickHand::kRightHand)) {
    //     flywheelspeed = FlywheelConstants::kTrenchRPM;
    //     hoodangle = HoodConstants::kTrenchPosition;
    //     if (Util::Deadzone(sqrt(pow(turretXRot, 2) + pow(turretYRot, 2)), OIConstants::kDeadzoneAbsRot) == 0) {
    //         turretXRot = 0;
    //         turretYRot = 0;
    //     }
    //     double rotPosition = atan2f(turretYRot, turretXRot);
    //     rotPosition *= 360.0/Math::kTau; 
    //     m_turret->TurnToRobot(rotPosition);
    // }
    // else {
        //angleOverride = turretXRot * TurretConstants::kMaxOverrideAngle;
        m_turret->TurnToRelative(m_vision.GetHubAngle() + angleOverride);
    // }

    //flywheelspeed *= FlywheelConstants::kHomingRPMMultiplier;
    //if (*m_firing)
    //    flywheelspeed *= FlywheelConstants::kFiringRPMMultiplier;
    
    m_hood->Set(hoodangle);
    m_flywheel->SetRPM(flywheelspeed);

    SmartDashboard::PutBoolean("D_FIRE_AT_RPM", m_flywheel->IsAtRPM());
    SmartDashboard::PutBoolean("D_FIRE_AT_SET", m_turret->isAtSetpoint());
    SmartDashboard::PutNumber("Hood Angle:", hoodangle);

    // if running manual trench fire
    // if (m_controller->GetBumper(GenericHID::JoystickHand::kRightHand))
    // {
    //     // if call to launch
    //     if (m_controller->GetYButtonPressed())
    //     {
    //         *m_turretready = true;
    //     }
    // }
    // // if at position, set turret ready to true
    // else 
    if (m_flywheel->IsAtRPMPositive())
    {
        *m_turretready = true;
    }
}

bool HomeTarget::IsFinished()
{
    // SmartDashboard::PutBoolean("TEST_FIRE_FINISIHED", *m_finished);
    return *m_finished;
}

void HomeTarget::End(bool interrupted) {
    *m_finished = false;
    *m_turretready = false;
    m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
    m_hood->Set(HoodConstants::kMax);
    m_turret->TurnTo(TurretConstants::kStartingPositionDegrees);
}