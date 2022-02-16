#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include <frc/DigitalInput.h>

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#include "Constants.h"
#include "common/Util.h"

class TransportSubsystem : public frc2::SubsystemBase
{
public:
    TransportSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Turns the Transfer at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetTransfer(double speed);
    
    /// Turns the Feeder at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetFeeder(double speed);

    /// Begins parallel thread detection of input
    void StartDetection();
    /// Begins parallel thread detection of input
    void EndDetection();

    /// \return Whether or not the transfer sensor is being blocked
    bool AtTransferPosition();

     /// \return Whether or not the transfer sensor is being blocked
    bool AtFeederPosition();

    bool GetTransferPhotoeye();

    bool GetFeederPhotoeye();

private:
    /// 775 that feeds balls from transfer into shooter
    TalonSRX m_feedermotor;
    /// 775 that shuffles balls around in the transfer system
    TalonSRX m_transfermotor;
    /// Photoeye to sense the ball at the feeder motor
    frc::DigitalInput m_feederphotoeye;
    /// Photoeye to sense the ball in the transfer system
    frc::DigitalInput m_transferphotoeye;
};