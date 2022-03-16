/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"

#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>

#include <wpi/numbers>

#include <ctre/phoenix/CANifier.h>

using namespace ctre::phoenix;
using namespace units;

/// Uncomment to set button binds for secondary controller to the primary controller
#define DualJoysticks

namespace DriveConstants
{
    constexpr int kNumSwerveModules = 4;

    /// Distance between centers of left and right wheels on robot
    constexpr meter_t kTrackWidth = 23.5_in;
    /// Distance between centers of front and back wheels on robot
    constexpr meter_t kWheelBase = 23.5_in;

    /// \name Teleop Drive Constraints
    // constexpr auto kDriveSpeed = meters_per_second_t(1.5);
    //constexpr auto kDriveSpeed = meters_per_second_t(4.0);
    constexpr meters_per_second_t kDriveSpeed = feet_per_second_t(13.6);

    constexpr auto kSlowDriveSpeed = meters_per_second_t(0.8);
    constexpr auto kDriveAngularSpeed = radians_per_second_t(wpi::numbers::pi * 2.0);
    constexpr auto kSlowDriveAngularSpeed = radians_per_second_t(wpi::numbers::pi / 2.0);

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

    //#define OFFSET_CONSTANTS_ZERO // Define this for callbrating the offsets
    #ifdef OFFSET_CONSTANTS_ZERO
    //============================================LEAVE THESE ZEROES COMMENTED OUT!!!
    constexpr double kFrontLeftOffset   = 0.0;
    constexpr double kFrontRightOffset  = 0.0;
    constexpr double kRearRightOffset   = 0.0;
    constexpr double kRearLeftOffset    = 0.0;
    //===============================================================================
    #else
    // Offsets updated on 2022 Feb 21 after Sussex
    constexpr double kFrontLeftOffset   = 1795.0; //1791; 
    constexpr double kFrontRightOffset  = 2543.0; ///2509.0;
    constexpr double kRearRightOffset   = 2030.0; //904.0;
    constexpr double kRearLeftOffset    = 2652.0; // 2649.0;
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

namespace ModuleConstants
{
    constexpr int kEncoderCPR = 2048;

    constexpr int kEncoderTicksPerSec = 10;                 //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    constexpr double kWheelDiameterMeters = .1016;          //!< 4"

    constexpr double kDriveGearRatio = 8.16;                //!< MK3 swerve modules w/NEOs 12.1 ft/sec w/Falcon 13.6 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 12.8;
    /// Assumes the encoders are directly mounted on the wheel shafts
    /// ticks / 100 ms -> ticks / s -> motor rev / s -> wheel rev / s -> m / s
    constexpr double kDriveEncoderMetersPerSec = kEncoderTicksPerSec / static_cast<double>(kEncoderCPR) / kDriveGearRatio * (kWheelDiameterMeters * wpi::numbers::pi);

    constexpr double kTurnEncoderCPR = 4096.0 / kTurnMotorRevsPerWheelRev;    // Mag encoder relative output to SparkMax

    constexpr double kP_ModuleTurningController = 1.1;

    constexpr double kD_ModuleTurningController = 0.03;
    constexpr double kPModuleDriveController = 0.001;

    constexpr uint kMotorCurrentLimit = 30;

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
    constexpr double kDriveP = 0.0025; // 0.1;
    constexpr double kDriveI = 0;
    constexpr double kDriveD = 0;
    constexpr double kDriveFF = 0.055;//0.047619;
    ///@}
}

namespace AutoConstants
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

    //constexpr auto kMaxSpeed = meters_per_second_t(2.8);
    constexpr meters_per_second_t kMaxSpeed = meters_per_second_t(3.6); // feet_per_second_t(13.6);
    constexpr auto kIntakeDriveSpeed = meters_per_second_t(0.5);

    constexpr auto kMaxAcceleration = meters_per_second_squared_t(1.0);

    // constexpr auto kMaxSpeed = meters_per_second_t(2.0);
    // constexpr auto kMaxAcceleration = meters_per_second_squared_t(3.0);

    //constexpr auto kMaxSpeed = meters_per_second_t(3.75);
    //constexpr auto kMaxAcceleration = meters_per_second_squared_t(4.5);

    constexpr auto kMaxAngularSpeed = radians_per_second_t(wpi::numbers::pi);
    constexpr auto kMaxAngularAcceleration = 4*unit_t<radians_per_second_squared_t>(wpi::numbers::pi);
    // constexpr auto kMaxAngularSpeed = radians_per_second_t(wpi::numbers::pi * 6.0);
    // constexpr auto kMaxAngularAcceleration = unit_t<radians_per_second_squared_t>(wpi::numbers::pi * 6.0);

    constexpr double kPXController = 1.0; // 1.5; // 0.5;//20; //7.0;
    constexpr double kIXController = 0;//1; 
    constexpr double kDXController = 0;//1; // 0.7;
    constexpr double kPYController = kPXController; //7.0;
    constexpr double kIYController = kIXController; 
    constexpr double kDYController = kDXController; // 0.7;
    constexpr double kPThetaController = 2.0; //0.5; //20; // 10.0;
    constexpr double kIThetaController = 0.1; 
    constexpr double kDThetaController = 1; // 0.9;

    extern const frc::TrapezoidProfile<radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants
{
    constexpr double kDeadzoneX = 0.015;
    constexpr double kDeadzoneY = 0.015;
    constexpr double kDeadzoneXY = 0.08;
    constexpr double kDeadzoneRot = 0.10;
    constexpr double kDeadzoneAbsRot = 0.50;

    constexpr int kPrimaryControllerPort = 0;
#ifdef DualJoysticks
    constexpr int kSecondaryControllerPort = 1;
#else
    constexpr int kSecondaryControllerPort = 0;
#endif
}

// Vision Subsystem Constants
namespace VisionConstants
{
    constexpr units::meter_t kHubOffsetRimToCenter = units::foot_t(2.5);  // Duluth adjustment to leesen dist by 1 ft
    constexpr units::meter_t kTargetDistIntoHub = units::foot_t(2.0);     // Separate offset from target dist within cone

    // constexpr frc::Translation2d kHubCenter = frc::Translation2d(kFieldLength/2, kFieldWidth/2);
    // constexpr frc::Translation2d turretCenterToRobotCenter = frc::Translation2d(inch_t{2.25}, inch_t{0});
    // constexpr frc::Translation2d camToTurretCenter = frc::Translation2d(meter_t{(cos(angleTurret) * inch_t{-12})}, meter_t{(sin(angleTurret) * inch_t{-12})});
 
    constexpr int kVisionFailLimit = 5;
    // 6/30/21
    // Limelight X Offset: -0.04
    // Mounting angle of the limelight, in degrees
    // Permanent X adjustment -0.05
    // Mounting height of the limelight from the ground, in inches
    // Target center height, in inches
    // 6/30/21 Changed: Target bottom now instead for consistent tracking in worse conditions
   
    constexpr inch_t kCameraHeight = inch_t{40};
    constexpr inch_t kCurrTargetHeight = inch_t{8*12 + 7};
    constexpr degree_t kCameraPitch = degree_t{22};

    constexpr double kMinTargetDistance = 70;
    constexpr double kMaxTargetDistance = 380;

    constexpr double kMinHoneDistance = 130;
    constexpr double kMaxHoneDistance = 260;

    constexpr double kRangeSmoothing = 0.7;

    constexpr units::meter_t kFieldLength = 648.0_in;
    constexpr units::meter_t kFieldWidth = 324.0_in;
    constexpr units::meter_t kHangarLength = 128.75_in;
    constexpr units::meter_t kHangarWidth = 116.0_in;

    //constexpr frc::Translation2d kHubCenter = Translation2d(kFieldLength/2, kFieldWidth/2);

    constexpr units::meter_t kVisionTargetDiameter = 53.375_in;
    constexpr units::meter_t kVisionTargetHeight = 77.5_in;

    constexpr units::meter_t kVisionTargetRadius = kVisionTargetDiameter / 2; //Should be meters type
    constexpr units::meter_t kMaxTargetSpread = 1.1 * kVisionTargetRadius;
}

// Flywheel Subsystem constants
namespace FlywheelConstants
{
    constexpr int kPrimaryMotorPort = 11;     //!< Flywheel CAN ID (Primary SparkMAX)
    constexpr int kFollowerMotorPort = 12;            //!< Flywheel CAN ID (Following SparkMAX)

    constexpr double kRampRate = 1.0;
    // Total error allowed for the flywheel, in RPM
    constexpr double kAllowedError = 75;
    constexpr double kMaintainPIDError = 300;

    // General multiplier added, adjusts for ball conditions and general firing
    constexpr double kHomingRPMMultiplier = 1.0175;
    constexpr double kIdleHomingRPMMultiplier = 1.01;
    // Additional multiplier applied to flywheel speed while firing 
    // Ensures all ball trajectories are straight
    constexpr double kFiringRPMMultiplier = 1.01; //TEMP 1.015; //2; //1.035; //1.05;

    // Launch PID values, used to first get to setpoint
    //constexpr double kP = 0.0002900;
    //constexpr double kP = 0.3;
    constexpr double kP = 0.0005;
//    constexpr double kP = 0.0;
    //constexpr double kI = 0.003;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;

    // Maintain PID values, used to adjust for error once the robot is shooting
    constexpr double kMP = 0.002000;
    constexpr double kMI = 0.00000001;
    constexpr double kMD = 0.000001;

    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 1.0;

    // constexpr double kS = 0.26625;  // Characterization should be repeated with 2 Neos
    // constexpr double kV = 0.12771;
    // constexpr double kA = 0.031171;
    constexpr double kS = 0.25701;  // Characterization for 2 Neos 2022 Feb 19
    constexpr double kV = 0.12024;
    constexpr double kA = 0.025359;

    // Diameter is in meters
    constexpr double kWheelDiameter = 0.1016;   // 4 inches
    constexpr double kSecondsPerMinute = 60;
    constexpr double kWheelMetersPerRev = kWheelDiameter * wpi::numbers::pi;
    // Meters per second to Revolutions per minute
    constexpr double kMPSPerRPM = kWheelMetersPerRev / kSecondsPerMinute;
        /// One turn of the Neo is 1.5 turns of the Flywheel
    constexpr double kGearRatio = 3.0 / 2.0;
    constexpr double kWheelRevPerMotorRev = kGearRatio;

    /// Use MPSPerRPM to determine the ramp rates, current values are just placeholders
    constexpr double kIdleRPM = 2000;
    /// The fixed RPM to fire at the trench given very heavy defense
    constexpr double kTrenchRPM = 3400;
}

// Intake Subsystem constants
namespace IntakeConstants
{
    constexpr int kMotorPort = 14;   // Intake rollers CAN ID (Talon)
    constexpr int kMotorReverseConstant = 1;
    constexpr int kSolenoidPort = 15;

    constexpr double kIngestSpeed = 0.80;
    constexpr double kReleaseSpeed = -0.80;
}

namespace TransferConstants
{
    constexpr int kFeederCANid = 12;      //!< Feeder CAN ID (TalonSRX)
    constexpr int kTransferCANid = 11;   //!< Transfer CAN ID (TalonSRX)

    constexpr int kFeederInputChannel = 0;
    constexpr int kTransferInputChannel = 1;

    constexpr double kFeederSpeedIntaking = 0.5;
    constexpr double kFeederSpeedFiring = 1.0;
    constexpr double kTransferSpeedIntaking = 0.5; //0.7;
    constexpr double kTransferSpeedFiring = 0.8;

    // Time to go from 0 to full throttle
    constexpr double kTransferRampRate = 0.75;

    constexpr double kTimePassed = 0.150;
    constexpr double kTimeLaunch = 1.50;

    constexpr double kTimeout = 30.0;
    constexpr bool kTransferInverted = false;
    constexpr bool kFeederInverted = true;
}

// Turret Subsystem Constants
namespace TurretConstants
{
    constexpr int kMotorPort = 13;   //!< Turret CAN ID (TalonSRX)

    // constexpr double kP = 0.3;
    // constexpr double kI = 0.001;
    // constexpr double kD = 10.0;
    constexpr double kP = 0.01;
    constexpr double kI = 0.001;
    constexpr double kD = 0.0;
    constexpr double kF = 0.45;

    constexpr double kNeutralDeadband = 0.06; // Deadband percentage
    constexpr double kMMCruiseVel = 300; // deg per sec 
    constexpr double kMMAccel = 100; // deg per sec^2
    constexpr double kMinOut = 0;
    constexpr double kMaxOut = 0.4; //1.0; //0.900;

    constexpr double kTimeout = 30;
    constexpr double kInverted = true;
    constexpr double kSensorPhase = true;

    constexpr double kDegreeStopRange = 0.85; //1; //1.35; //0.6; //0.4; //0.5;

    // Offset of origin point of turret angle and robot angle, in degrees. Robot 0 is forward
    constexpr double kTurretToRobotAngleOffset = 0.0;
    // Maximum rotation of the turret relative to the turret, in degrees
    constexpr double kMinAngle = -60.0;
    constexpr double kMaxAngle = 60.0;

    // initial configured angle of the turret relative to the turret, in degrees
    constexpr double kStartingPositionDegrees = 0.0;
}

//Hood Subsystem Constants
namespace HoodConstants
{
    /// PWM Port for hood servo
    constexpr int kPWMPort = 8;                //!< Hood servo PWM channel
    constexpr double kTestServoSpeed = 0.14;
    // Drives from Max to Min, where hood is smallest at 0.85, and greatest at 0.0485
    constexpr double kMax = 0.75;
    constexpr double kMin = 0.19;

    /// The fixed hood to fire in the trench given very heavy defense
    constexpr double kTrenchPosition = 0.223;
}

// Climber Subsystem constants
namespace ClimberConstants
{
    constexpr double kMotorCanId = 15;          // Climber CAN ID TalonSRX
    constexpr double kMotorReverseConstant = -1;
    constexpr double kMotorSpeed = 1.0;
}

namespace CompressorConstans
{
    constexpr int kCompressorPort = 1;
}