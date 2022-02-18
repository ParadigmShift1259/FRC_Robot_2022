#pragma once

#include <frc/trajectory/TrapezoidProfile.h>    // For thetaController

#include <wpi/numbers>

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>

using namespace units;

/// Autonomous constants
namespace AutoConstants
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

    // constexpr auto kMaxSpeed = meters_per_second_t(1.0);
    // constexpr auto kMaxAcceleration = meters_per_second_squared_t(2.0);

    constexpr auto kMaxSpeed = meters_per_second_t(3.0);
    constexpr auto kMaxAcceleration = meters_per_second_squared_t(4.0);

    //constexpr auto kMaxSpeed = meters_per_second_t(3.75);
    //constexpr auto kMaxAcceleration = meters_per_second_squared_t(4.5);
    constexpr auto kMaxAngularSpeed = radians_per_second_t(wpi::numbers::pi * 6.0);
    constexpr auto kMaxAngularAcceleration = unit_t<radians_per_second_squared_t>(wpi::numbers::pi * 6.0);

    constexpr double kPXController = 7.0;
    constexpr double kDXController = 0.7;
    constexpr double kPYController = 7.0;
    constexpr double kDYController = 0.7;
    constexpr double kPThetaController = 10.0;
    constexpr double kDThetaController = 0.9;

    extern const frc::TrapezoidProfile<radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants
