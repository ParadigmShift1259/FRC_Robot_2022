#pragma once

#include <wpi/numbers>

// Flywheel subsystem constants
namespace FlywheelConstants
{
    constexpr int kPrimaryMotorPort = 11;     //!< Flywheel CAN ID (Primary SparkMAX)
    constexpr int kFollowerMotorPort = 12;    //!< Flywheel CAN ID (Following SparkMAX)

    constexpr double kRampRate = 1.0;
    // Total error allowed for the flywheel, in RPM
    constexpr double kAllowedError = 75.0;
    constexpr double kMaintainPIDError = 300.0;

    // General multiplier added, adjusts for ball conditions and general firing
    constexpr double kHomingRPMMultiplier = 1.0175;
    constexpr double kIdleHomingRPMMultiplier = 1.01;
    // Additional multiplier applied to flywheel speed while firing 
    // Ensures all ball trajectories are straight
    constexpr double kFiringRPMMultiplier = 1.01; //TEMP 1.015; //2; //1.035; //1.05;

    // Launch PID values, used to first get to setpoint
    constexpr double kP = 0.0002900;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;

    // Maintain PID values, used to adjust for error once the robot is shooting
    constexpr double kMP = 0.002000;
    constexpr double kMI = 0.00000001;
    constexpr double kMD = 0.000001;

    constexpr double kMinOut = 0.0;
    constexpr double kMaxOut = 1.0;

    constexpr double kS = 0.26625;  // Characterization should be repeated with 2 Neos
    constexpr double kV = 0.12771;
    constexpr double kA = 0.031171;

    // Diameter is in meters
    constexpr double kWheelDiameter = 0.1016;   // 4 inches
    constexpr double kSecondsPerMinute = 60.0;
    constexpr double kWheelMetersPerRev = kWheelDiameter * wpi::numbers::pi;
    // Meters per second to Revolutions per minute
    constexpr double kMPSPerRPM = kWheelMetersPerRev / kSecondsPerMinute;
    /// One turn of the Neo is 1.5 turns of the Flywheel
    constexpr double kGearRatio = 3.0 / 2.0;
    constexpr double kWheelRevPerMotorRev = kGearRatio;

    /// Use MPSPerRPM to determine the ramp rates, current values are just placeholders
    constexpr double kIdleRPM = 2000.0;
    /// The fixed RPM to fire at the trench given very heavy defense
    constexpr double kTrenchRPM = 3400.0;
}
