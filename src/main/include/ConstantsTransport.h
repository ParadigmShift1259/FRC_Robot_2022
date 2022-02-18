#pragma once

/// Transport subsystem constants
namespace TransferConstants
{
    constexpr int kFeederCANid = 12;      //!< Feeder CAN ID (TalonSRX)
    constexpr int kTransferCANid = 11;    //!< Transfer CAN ID (TalonSRX)

    constexpr int kFeederInputChannel = 0;
    constexpr int kTransferInputChannel = 1;

    constexpr double kFeederSpeed = 0.5;
    constexpr double kSpeedFiring = 0.7;
    constexpr double kTransferSpeedIntaking = 0.500;

    // Time to go from 0 to full throttle
    constexpr double kTransferRampRate = 0.75;

    constexpr double kTimePassed = 0.25;
    constexpr double kTimeLaunch = 1.50;

    constexpr double kTimeout = 30.0;
    constexpr bool kTransferInverted = false;
    constexpr bool kFeederInverted = true;
}
