/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Filesystem.h>
#include <frc/XboxController.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc/geometry/Translation2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <wpi/fs.h>

#include "common/Util.h"
#include "Gyro.h"
#include "common/SwerveControllerCommand2.h"

#include "IOdometry.h"

#include "ISubsysAccess.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

#include "commands/TransferFirstBall.h"
#include "commands/TransferSecondBall.h"
#include "commands/IntakeTransfer.h"
#include "commands/IntakeIngest.h"
#include "commands/Unjam.h"
#include "commands/IntakeRelease.h"
#include "commands/Fire.h"

#include "Constants.h"

#include <iostream>
// #include <wpi/Path.h>
#include <wpi/SmallString.h>

#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;
using SwerveCtrlCmd = frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules>;

class RobotContainer : public ISubsysAccess, public IOdometry
{
public:
    RobotContainer();

    void Periodic();

    void ZeroDrive();
    void TurretSetZeroAngle() { m_turret.SetZeroAngle(); }
    void GyroSetZeroHeading() { m_gyro.ZeroHeading(); }

    enum EAutoPath {kEx1, kEx2, kEx3, kEx4, kEx5};
    frc2::Command *GetAutonomousCommand(EAutoPath path);

    frc::SendableChooser<EAutoPath> m_chooser;

    HoodSubsystem&       GetHood() override { return m_hood; };
    IntakeSubsystem&     GetIntake() override { return m_intake; };
    TransferSubsystem&   GetTransfer() override { return m_transfer; };
    TurretSubsystem&     GetTurret() override { return m_turret; };
    VisionSubsystem&     GetVision() override { return m_vision; };    

    Pose2d GetPose() { return m_drive.GetPose(); }
    Pose2d GetPose(units::time::second_t timestamp) const { return m_drive.GetPose(timestamp); }
    const vector<frc::Trajectory::State>& GetStateHist() const { return m_drive.GetStateHist(); }
    void ResetOdometry(frc::Pose2d pose) { m_drive.ResetOdometry(pose); }

    double GetYvelovity() { return m_drive.GetYvelocity().to<double>(); }

private:
    void SetDefaultCommands();
    void ConfigureButtonBindings();
    frc2::SequentialCommandGroup* GetAutoPathCmd(frc::Trajectory trajectory, bool primaryPath);
    SwerveCtrlCmd GetSwerveCommandPath(frc::Trajectory trajectory, bool primaryPath);
    frc::Trajectory convertPathToTrajectory(PathPlannerTrajectory path);
    void PrintTrajectory(frc::Trajectory& trajectory);

    frc::XboxController m_primaryController{OIConstants::kPrimaryControllerPort};
    frc::XboxController m_secondaryController{OIConstants::kSecondaryControllerPort};

    Team1259::Gyro m_gyro;
    DriveSubsystem m_drive;
    bool m_fieldRelative = true;
    VisionSubsystem m_vision; 
    FlywheelSubsystem m_flywheel;
    IntakeSubsystem m_intake;
    TransferSubsystem m_transfer;
    TurretSubsystem m_turret = TurretSubsystem(&m_gyro);
    HoodSubsystem m_hood;
    ClimberSubsystem m_climber;

    frc2::InstantCommand m_setFieldRelative{[this] { m_fieldRelative = true; }, {}};
    frc2::InstantCommand m_clearFieldRelative{[this] { m_fieldRelative = false; }, {}};
    frc2::InstantCommand m_toggleMaxDriveSpeed
    {[this]
        { 
            m_bLowSpeedDriving = !m_bLowSpeedDriving;
            SmartDashboard::PutBoolean("LowSpeedDriveing", m_bLowSpeedDriving);
            m_maxRotSpeed = m_bLowSpeedDriving ? kSlowDriveAngularSpeed : kDriveAngularSpeed;
            m_drive.SetMaxDriveSpeed(m_bLowSpeedDriving ? kSlowDriveSpeed : kDriveSpeed);
        },
        {&m_drive}
    };
    frc2::InstantCommand m_zeroHeading{[this] { m_gyro.ZeroHeading(); }, {}};
    frc2::InstantCommand m_climb{[this] { m_climber.Run(ClimberConstants::kMotorSpeed); }, {&m_climber} };
#define CLIMB_TEST_DO_NOT_USE_WITH_RACTHET
#ifdef CLIMB_TEST_DO_NOT_USE_WITH_RACTHET
    frc2::InstantCommand m_windClimb{[this] { m_climber.Run(-1.0 * ClimberConstants::kMotorSpeed); }, {&m_climber} };
#endif

    double m_overrideAngle = 0.0;

    bool m_turretready = false;
    bool m_firing = false;
    bool m_finished = false;
    bool m_bLowSpeedDriving = false;
    radians_per_second_t m_maxRotSpeed { kDriveAngularSpeed };

    DebugFlag   m_dbgSeroTest{"ServoTest", false};
    
    
    frc2::SequentialCommandGroup* GetBall1Cmd(void);
    void BuildTrajectories(void);
    // Trajectory m_ball1Traj;
    Trajectory m_ball1TrajPt1;
    Trajectory m_ball1TrajPt2;
    Trajectory m_ball1TrajPt3;
};
