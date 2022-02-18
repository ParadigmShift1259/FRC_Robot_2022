#pragma once

//#include <units/length.h>

//using namespace units;

// Vision subsystem constants
namespace VisionConstants
{
    // 6/30/21
    // Limelight X Offset: -0.04
    // Mounting angle of the limelight, in degrees
    constexpr double kMountingAngle = 25.0;
    // Permanent X adjustment -0.05
    // Mounting height of the limelight from the ground, in inches
    constexpr double kMountingHeight = 22.0;
    // Target center height, in inches
    // 6/30/21 Changed: Target bottom now instead for consistent tracking in worse conditions
    constexpr double kTargetHeight = 81.25;  //98.25;

    constexpr double kMinTargetDistance = 70.0;
    constexpr double kMaxTargetDistance = 380.0;

    constexpr double kMinHoneDistance = 130.0;
    constexpr double kMaxHoneDistance = 260.0;

    constexpr double kHubRadius = 0.601; //Should be meters type
    constexpr double kMaxTargetSpread = 1.1 * kHubRadius;
}
