
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>

#include "Constants.h"
#include "Calculations.h"

using namespace VisionConstants;

const double hahLow = 9.2;
const double hahHigh = 9.8; //10.0;

const double nearDist = 4.0;
const double farDist = 15.0;

const double slope = (hahLow - hahHigh) / (farDist - nearDist);
const double offset = hahHigh - slope * nearDist;

class HoodSubsystem : public frc2::SubsystemBase
{
public:
    HoodSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Set hood to certain position
    /// \param position         Servo rotation, ranging from [0, 1]
    void SetServoPosition(double position);

    /// Set hood to certain position
    /// \param distHubCenter    Distance to center of hub
    void SetByDistance(double distHubCenter);

    double GetFlywheelSpeed() { return m_flywheelSpeed; }

private:
    /// Servo that moves hood up and down
    frc::Servo m_servo;
   	Calculations m_calculation;
    double m_flywheelSpeed;
};
