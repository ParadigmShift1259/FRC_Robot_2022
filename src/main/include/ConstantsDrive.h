#pragma once

#include <wpi/numbers>

#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <ctre/phoenix/CANifier.h>

using namespace ctre::phoenix;
using namespace units;

/// Drive subsystem constants
namespace DriveConstants
{
    constexpr int kNumSwerveModules = 4;

    /// Distance between centers of left and right wheels on robot
    constexpr meter_t kTrackWidth = 23.5_in;
    /// Distance between centers of front and back wheels on robot
    constexpr meter_t kWheelBase = 23.5_in;

    /// \name Teleop Drive Constraints
    constexpr auto kDriveSpeed = meters_per_second_t(1.5);
    //constexpr auto kDriveSpeed = meters_per_second_t(3.5);
    constexpr auto kDriveAngularSpeed = radians_per_second_t(wpi::numbers::pi * 2.0);

    /// \name CAN bus IDs
    ///@{
    /// CAN IDs for swerve modules
    constexpr int kCanifierID = 0;                       //!< CANifier CAN ID (for absolute encoder PWM inputs)
    
    constexpr int kFrontLeftDriveMotorPort    = 1;       //!< Front Left Drive CAN ID (TalonFX)   
    constexpr int kFrontLeftTurningMotorPort  = 2;       //!< Front Left Turn CAN ID (SparkMAX)   

    constexpr int kFrontRightDriveMotorPort   = 3;       //!< Front Right Drive CAN ID (TalonFX)   
    constexpr int kFrontRightTurningMotorPort = 4;       //!< Front Right Turn CAN ID (SparkMAX)

    constexpr int kRearRightDriveMotorPort    = 5;       //!< Rear Right Drive CAN ID (TalonFX)   
    constexpr int kRearRightTurningMotorPort  = 6;       //!< Rear Right Turn CAN ID (SparkMAX)

    constexpr int kRearLeftDriveMotorPort     = 7;       //!< Rear Left Drive CAN ID (TalonFX)   
    constexpr int kRearLeftTurningMotorPort   = 8;       //!< Rear Left Turn CAN ID (SparkMAX)
    ///@}

    /// \name Canifier PWM channels
    ///@{
    /// PWM channels for the canifier
    constexpr CANifier::PWMChannel kFrontLeftPWM = CANifier::PWMChannel::PWMChannel0;
    constexpr CANifier::PWMChannel kFrontRightPWM = CANifier::PWMChannel::PWMChannel2;
    constexpr CANifier::PWMChannel kRearRightPWM = CANifier::PWMChannel::PWMChannel1;
    constexpr CANifier::PWMChannel kRearLeftPWM = CANifier::PWMChannel::PWMChannel3;
    ///@}

    /// \name Drive wheel reversal (inverting) flags
    ///@{
    /// To keep the swerve module bevel gear facing inwards we need to reverse the right side
    constexpr bool kFrontLeftDriveMotorReversed  = true;
    constexpr bool kRearLeftDriveMotorReversed   = true;
    constexpr bool kFrontRightDriveMotorReversed = false;
    constexpr bool kRearRightDriveMotorReversed  = false;
    ///@}

    constexpr bool kGyroReversed = false;

    // Process for reentering values: 0 all values out, line up with stick, all gears face inwards
    // Line up based on side, left or right
    // Record values, enter below, then redeploy
    // All gears should face outwards

    //#define OFFSET_CONSTANTS_ZERO

    #ifdef OFFSET_CONSTANTS_ZERO
    //============================================LEAVE THESE ZEROES COMMENTED OUT!!!
    constexpr double kFrontLeftOffset   = 0.0;
    constexpr double kFrontRightOffset  = 0.0;
    constexpr double kRearRightOffset   = 0.0;
    constexpr double kRearLeftOffset    = 0.0;
    //===============================================================================
    #else
    // Offsets set on 2022 swerve sled 2022 Feb 5
    constexpr double kFrontLeftOffset   = 1776.0; // 2701.0; // 2700.0; //This is Good //Encoder 2.07 Radians //2689.0;
    constexpr double kFrontRightOffset  = 2550.0; // 190.0; // 188.0; //This is Good //1541.0; //Encoder 3.84 Radians //205.0;
    constexpr double kRearRightOffset   = 911.0; // 1863.0; // 3077.0; //Encoder 2.08 Radians //1858.0;
    constexpr double kRearLeftOffset    = 2683.0; // 1029.0; // 2007.0; //Encoder 2.04 Radians //983.0;
    #endif

    // Pulse Width per rotation is not equal for all encoders. Some are 0 - 3865, some are 0 - 4096
    // FL: 4096
    // FR: 3970
    // RL: 4096
    // RR: 3865
    constexpr double kPulseWidthToZeroOne = 4096.0;    // 4096 micro second pulse width is full circle
    constexpr double kPulseWidthToRadians =  2.0 * wpi::numbers::pi / kPulseWidthToZeroOne;

    /// \name Robot RotationDrive PID Controller
    ///@{
    /// Rotation PID Controller for Rotation Drive, converts between radians angle error to radians per second turn
    constexpr double kRotationDriveP = 1;
    constexpr double kRotationDriveI = 0;
    constexpr double kRotationDriveIMaxRange = 0;
    constexpr double kRotationDriveD = 0.025;
    /// Max speed for control
    constexpr double kRotationDriveMaxSpeed = 3.5;
    /// Speeds higher than value will prevent robot from changing directions for a turn
    constexpr double kRotationDriveDirectionLimit = 3;
    /// Tolerance for turning
    constexpr double kRotationDriveTolerance = 0.07;
    ///@}
}
