/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/CANifier.h>

#include "common/Util.h"
#include "Gyro.h"

#include "Constants.h"
#include "SwerveModule.h"
#include <vector>
#include <frc/trajectory/Trajectory.h>

// Uncomment to directly set states to each module
//#define MANUAL_MODULE_STATES
// Uncomment to tune Rotation Drive PIDs
//#define TUNE_ROTATION_DRIVE

using namespace ctre::phoenix;
using namespace DriveConstants;
using namespace std;
using namespace frc;

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    enum ModuleLocation    //!< Order as returned by kDriveKinematics.ToSwerveModuleStates
    {
        kFrontLeft,
        kFrontRight,
        kRearLeft,
        kRearRight
    };

    DriveSubsystem(Team1259::Gyro *gyro);

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    // Subsystem methods go here.

    /// Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
    /// and the linear speeds have no effect on the angular speed.
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angular rate of the robot.
    /// \param fieldRelative Whether the provided x and y speeds are relative to the field.
    /// \param isAuto          Whether the bot is using function for auto or not. False by default. 
    void Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative);

    // Drives the robot with the right stick controlling the position angle of the robot
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angle of the robot in radians
    /// \param fieldRelative Whether the provided translational speeds are relative to the field.
    void RotationDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radian_t rot, bool fieldRelative);

    // Drives the robot with the right stick controlling the position angle of the robot
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param xRot          Angle of the robot on the x axis
    /// \param yRot          Angle of the robot on the y axis
    /// \param fieldRelative Whether the provided translational speeds are relative to the field.
    void RotationDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, double xRot, double yRot, bool fieldRelative);

    /// Drives the robot and maintains robot angle with no rotational input
    ///
    /// \param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// \param ySpeed        Speed of the robot in the y direction (sideways).
    /// \param rot           Angular rate of the robot.
    /// \param fieldRelative Whether the provided x and y speeds are relative to the field.
    void HeadingDrive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative);

    /// Updates the last heading set for Heading Drive. Needs to be called if transitioning from other Drive functions to HeadingDrive
    void UpdateLastHeading();

    /// Resets the drive encoders to currently read a position of 0.
    void ResetEncoders();

    /// Readable alias for array of swerve modules
    using SwerveModuleStates = wpi::array<SwerveModuleState, DriveConstants::kNumSwerveModules>;
    /// Sets the drive SpeedControllers to a power from -1 to 1.
    void SetModuleStates(SwerveModuleStates desiredStates);

    /// Returns the currently-estimated pose of the robot.
    /// \return The pose.
    Pose2d GetPose();

    /// Returns the previous estimated pose of the robot at the given timestamp.
    /// \return The pose.
    Pose2d GetPose(units::time::second_t timestamp) const;

    /// Converts PWM input on the CANifier to a pulse width
    /// \param pwmChannel The PWM channel to pass in
    /// \return The pulse width of the PWM channel
    double PWMToPulseWidth(CANifier::PWMChannel pwmChannel);

    /// Resets the odometry to the specified pose.
    /// \param pose The pose to which to set the odometry.
    void ResetOdometry(Pose2d pose);

    /// Set all 4 wheels to the zero position
    void WheelsForward();

    /// Resync all relative NEO turn encoders to the absolute encoders
    void ResetRelativeToAbsolute();

    void SetMaxDriveSpeed(meters_per_second_t maxDriveSpeed) { m_maxDriveSpeed = maxDriveSpeed; }
    meters_per_second_t GetYvelocity() const { return m_yVelocity; }

    const vector<frc::Trajectory::State>& GetStateHist() const { return m_StateHist; }

    /// The kinematics object converts inputs into 4 individual swerve module turn angle and wheel speeds
    SwerveDriveKinematics<kNumSwerveModules> kDriveKinematics{
        Translation2d( kWheelBase / 2,  kTrackWidth / 2),    // +x, +y FL
        Translation2d( kWheelBase / 2, -kTrackWidth / 2),    // +x, -y FR
        Translation2d(-kWheelBase / 2,  kTrackWidth / 2),    // -x, +y RL
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2)};   // -x, -y RR

private:    
    /// Get all 4 swerve module wheel speed to update the odometry with
    SwerveModuleStates getCurrentWheelSpeeds()
    {
        SwerveModuleStates sms{
            m_frontLeft.GetState(),
            m_frontRight.GetState(),
            m_rearLeft.GetState(),
            m_rearRight.GetState()
        };
        return sms;
    }

    /// \name Swerve Modules
    /// The drive subsystem owns all 4 swerve modules
    ///@{
    SwerveModule m_frontLeft;
    SwerveModule m_frontRight;
    SwerveModule m_rearRight;
    SwerveModule m_rearLeft;
    ///@}

    /// Reads the absolute encoder pulse widths
    CANifier m_canifier;
    /// Gyro to determine field relative driving, from @ref RobotContainer
    Team1259::Gyro *m_gyro;
    /// Odometry class for tracking robot pose
    SwerveDriveOdometry<DriveConstants::kNumSwerveModules> m_odometry;

    /// PID to control overall robot chassis rotation 
    frc2::PIDController m_rotationPIDController{
        DriveConstants::kRotationDriveP,
        DriveConstants::kRotationDriveI,
        DriveConstants::kRotationDriveD
    };
    /// Last maintained heading, used for @ref HeadingDrive
    double m_lastHeading;
    /// Whether or not rotation input was provided, used for @ref HeadingDrive
    bool m_rotationalInput;
    Timer m_timer;
    vector<frc::Trajectory::State> m_StateHist;  // TO DO: change to new class derived from Trajectory::State with additional field for Turrent angle 
    double m_velocity;
    double m_acceleration;
    meters_per_second_t m_maxDriveSpeed { kDriveSpeed };
    meters_per_second_t m_yVelocity {0.0};
};
