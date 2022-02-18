#pragma once

#include <wpi/numbers>

/// Swerve module constants
namespace ModuleConstants
{
    constexpr int kEncoderCPR = 2048;

    constexpr int kEncoderTicksPerSec = 10;                 //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    constexpr double kWheelDiameterMeters = 0.1016;         //!< 4"

    constexpr double kDriveGearRatio = 8.16;                //!< MK3 swerve modules w/NEOs 12.1 ft/sec w/Falcon 13.6 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 12.8;
    /// Assumes the encoders are directly mounted on the wheel shafts
    /// ticks / 100 ms -> ticks / s -> motor rev / s -> wheel rev / s -> m / s
    constexpr double kDriveEncoderMetersPerSec = kEncoderTicksPerSec / static_cast<double>(kEncoderCPR) / kDriveGearRatio * (kWheelDiameterMeters * wpi::numbers::pi);

    constexpr double kTurnEncoderCPR = 4096.0 / kTurnMotorRevsPerWheelRev;    // Mag encoder relative output to SparkMax

    constexpr double kP_ModuleTurningController = 1.1;

    constexpr double kD_ModuleTurningController = 0.03;
    constexpr double kPModuleDriveController = 0.001;

    constexpr unsigned int kMotorCurrentLimit = 30;

    /// \name Turn PID Controller for Swerve Modules
    ///@{
    constexpr bool kTurnAdjust = false;
    constexpr double kTurnP = 0.75;
    constexpr double kTurnI = 0.0;
    constexpr double kTurnD = 0.0;
    constexpr double kTurnIA = 0.0;
    constexpr double kTurnIZ = 0.0;
    ///@}

    /// \name Drive PID Controller for Swerve Modules
    ///@{
    constexpr bool kDriveAdjust = false;
    constexpr double kDriveP = 0.1;
    constexpr double kDriveI = 0;
    constexpr double kDriveD = 0;
    constexpr double kDriveFF = 0.047619;
    ///@}
}
