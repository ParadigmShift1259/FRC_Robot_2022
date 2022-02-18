#pragma once

#include <wpi/numbers>

// Turret subsystem constants
namespace TurretConstants
{
    constexpr int kMotorPort = 13;   //!< Turret CAN ID (TalonSRX)

    constexpr double kP = 0.30114;
    constexpr double kI = 0.00035;
    constexpr double kD = 19.6;

    constexpr double kMinOut = 0.0;
    constexpr double kMaxOut = 0.700;

    constexpr double kTimeout = 30.0;
    constexpr double kInverted = true;
    constexpr double kSensorPhase = true;

    constexpr double kMaxOverrideAngle = 5.0; //10.0;

    constexpr double kDegreeStopRange = 0.85; //1; //1.35; //0.6; //0.4; //0.5;
    constexpr double kDegreePIDStopRange = 0.25; //0.35; //0.35;

    constexpr double kPulley = 2.7305;
    constexpr double kSpinner = 29.845;

    // The motor on the turret drives a pulley, while drives the turret
    // MotorRev indicates the revolution of the motor, while Rev indicates the revolution of the turret
    constexpr double kMotorRevPerRev = kPulley / kSpinner;
    constexpr double kTicksPerRev = 4096.0;
    constexpr double kDegreesPerRev = 360.0;
    constexpr double kRadiansPerRev = wpi::numbers::pi * 2.0;

    // Offset of origin point of turret angle and robot angle, in degrees. Robot 0 is forward
    constexpr double kTurretToRobotAngleOffset = -45.0;
    // Maximum rotation of the turret relative to the turret, in degrees
    constexpr double kMinAngle = 0.0;
    constexpr double kMaxAngle = 90.0;
    // Range of angle allowed for auto targeting by default
    constexpr double kMinAutoAngle = 25.0;
    constexpr double kMaxAutoAngle = 65.0;
    // Maximum relative angle allowed for auto targeting by default
    constexpr double kMaxAutoRelAngle = 20.0;

    // initial configured angle of the turret relative to the turret, in degrees
    constexpr double kStartingPositionDegrees = 45.0;
}
