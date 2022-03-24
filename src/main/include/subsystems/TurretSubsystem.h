
#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/AnalogInput.h>

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#include "Constants.h"
#include "common/Util.h"
#include "common/DebugFlag.h"
#include "Gyro.h"

class TurretSubsystem : public frc2::SubsystemBase
{
public:
    TurretSubsystem(Team1259::Gyro *gyro);

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;
    
    /// Set the current angle to zero
    /// Used to reset the straigt ahead angle (physically turn the turret forward)
    void SetZeroAngle();

    /// Turns the turret to a specified angle on the turret
    /// Most calculations currently depend on counter clockwise turning, with 0 as "front"
    /// \param angle        Angle desired to turn to, must be within the boundaries of the turret, must be positive, in degrees
    void TurnTo(double angle, double minAngle=TurretConstants::kMinAngle, double maxAngle=TurretConstants::kMaxAngle);

    /// Turns the turret based on an absolute field angle
    /// \param deisredAngle        Field angle to turn to, must be positive
    void TurnToField(double desiredAngle);

    /// Turns the turret to an angle added to the current robot position
    /// \param angle        Angle that should be added to the robot position and turned to, can be either positive or negative, in degrees
    void TurnToRelative(double angle, double minAngle=TurretConstants::kMinAngle, double maxAngle=TurretConstants::kMaxAngle);

    /// Returns whether or not the turret is at the desired setpoint
    bool isAtSetpoint();

    double GetCurrentAngle();

protected:
    /// Converts motor ticks into turret rotation, in degrees
    /// \param ticks        Number of ticks to be converted
    double TicksToDegrees(double ticks);

    /// Converts turret rotation in degrees into motor ticks
    /// \param degrees      Number of degrees to be converted
    double DegreesToTicks(double degrees);

    int GetAbsEncValue() { return 4096 - m_absEnc.GetValue();}
private:
    /// 775 that rurns the shooting mechanism
    TalonSRX m_turretmotor;
    /// The current angle of the turret, in degrees
    double m_currentAngle;
    /// The starting position of the turret, in ticks
    int m_startingPos;
    
    bool m_setZero = true; //Disabled auto turret centering due to new abs encoder 3-23-22 AC
    /// Gyro to determine field relative angles, from @ref RobotContainer
    Team1259::Gyro *m_gyro;

    frc::AnalogInput m_absEnc;

    DebugFlag   m_dbgLogTurns{"TurretLogTurn", false};
};