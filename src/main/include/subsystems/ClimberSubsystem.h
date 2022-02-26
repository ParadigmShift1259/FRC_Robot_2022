#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class ClimberSubsystem : public frc2::SubsystemBase
{
public:

    ClimberSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the climber at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Run(double speed);

private:
    /// 775 that pulls/releases rope for hooks
    TalonSRX m_motor;
};
