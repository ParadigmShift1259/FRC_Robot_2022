#pragma once

// Intake subsystem constants
namespace IntakeConstants
{
    constexpr int kMotorPort = 14;   			//!< Intake rollers CAN ID (Talon)
    constexpr int kMotorReverseConstant = 1;

    constexpr double kIngestLow = 0.3;
    constexpr double kIngestHigh = 0.80;
    constexpr double kReleaseLow = -0.3;
    constexpr double kReleaseHigh = -0.70;
}
