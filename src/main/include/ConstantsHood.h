#pragma once

// Hood subsystem constants
namespace HoodConstants
{
    /// PWM Port for hood servo
    constexpr int kPWMPort = 8;                //!< Hood servo PWM channel
	
    constexpr double kTestServoSpeed = 0.14;
    // Drives from Max to Min, where hood is smallest at 0.85, and greatest at 0.0485
    constexpr double kMax = 0.95;
    constexpr double kMin = 0.20;

    /// The fixed hood to fire in the trench given very heavy defense
    constexpr double kTrenchPosition = 0.223;
}