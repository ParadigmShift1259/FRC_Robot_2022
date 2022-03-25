/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Filesystem.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>

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
#include <frc2/command/ParallelRaceGroup.h>

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
// using SwerveCtrlCmd = frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules>;
using SwerveCtrlCmd = frc2::SwerveControllerCommand<DriveConstants::kNumSwerveModules>;

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

    HoodSubsystem&       GetHood() override { return m_hood; }
    IntakeSubsystem&     GetIntake() override { return m_intake; }
    TransferSubsystem&   GetTransfer() override { return m_transfer; }
    TurretSubsystem&     GetTurret() override { return m_turret; }
    VisionSubsystem&     GetVision() override { return m_vision; }
    DriveSubsystem&      GetDrive() { return m_drive; }
    
    bool OnlyOneBall() override { return m_onlyOneBall; }
    void SetOneBallFlag() override {m_onlyOneBall = true;}

    Pose2d GetPose() override { return m_drive.GetPose(); }
    Pose2d GetPose(units::time::second_t timestamp) const override { return m_drive.GetPose(timestamp); }
    StateHist GetState() const override { return m_drive.GetState(); }
    StateHist GetState(units::time::second_t timestamp) const override { return m_drive.GetState(timestamp); }
    const StateHistColl& GetStateHist() const override { return m_drive.GetStateHist(); }
    units::degree_t GetTurretAngle() override { return units::degree_t(m_turret.GetCurrentAngle()); }
    void ResetOdometry(frc::Pose2d pose) override { m_drive.ResetOdometry(pose); }
    void AddVisionMeasurement(const Pose2d& visionRobotPose, units::second_t timestamp) override { m_drive.AddVisionMeasurement(visionRobotPose, timestamp);}
    void SetVisionMeasurementStdDevs( const wpi::array<double, 3>& visionMeasurementStdDevs) override { m_drive.SetVisionMeasurementStdDevs(visionMeasurementStdDevs); }

    //bool HasAutoRun() { return m_hasAutoRun; }

    double GetYvelovity() { return m_drive.GetYvelocity().to<double>(); }
    
    void CloseLogFile() { m_vision.CloseLogFile(); }

    bool OdoValid() {return m_drive.OdoValid();};

private:
    void SetDefaultCommands();
    void ConfigureButtonBindings();
    void ConfigPrimaryButtonBindings();
    void ConfigSecondaryButtonBindings();
    frc2::SequentialCommandGroup* GetIntakeAndFirePathCmd(Trajectory trajectory, bool primaryPath);
    frc2::ParallelRaceGroup* GetIntakePathCmd(Trajectory trajectory, bool primaryPath);
    frc2::SequentialCommandGroup* GetFirePathCmd(Trajectory trajectory, bool primaryPath);
    SwerveCtrlCmd GetSwerveCommandPath(Trajectory trajectory, bool primaryPath);
    frc::Trajectory convertPathToTrajectory(PathPlannerTrajectory path);
    void PrintTrajectory(frc::Trajectory& trajectory);

    frc::XboxController m_primaryController{OIConstants::kPrimaryControllerPort};
    frc::XboxController m_secondaryController{OIConstants::kSecondaryControllerPort};

    Team1259::Gyro m_gyro;
    DriveSubsystem m_drive;
    bool m_fieldRelative = true;
    VisionSubsystem m_vision; 
    FlywheelSubsystem m_flywheel;
    frc::Compressor m_compressor;
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
    //frc2::InstantCommand m_zeroHeading{[this] { m_gyro.ZeroHeading(); }, {} };
    //frc2::InstantCommand m_setTurretZero{[this] { m_turret.SetZeroAngle(); }, {&m_turret} };
    frc2::InstantCommand m_climb{[this]
    {
        m_vision.SetLED(false);
        m_turret.TurnTo(0.0);
        m_climber.Run(ClimberConstants::kMotorSpeed); }, {&m_climber}
    };
//#define CLIMB_TEST_DO_NOT_USE_WITH_RACTHET
#ifdef CLIMB_TEST_DO_NOT_USE_WITH_RACTHET
    frc2::InstantCommand m_windClimb{[this] { m_climber.Run(-1.0 * ClimberConstants::kMotorSpeed); }, {&m_climber} };
#endif
    frc2::InstantCommand m_toggleVisionMode{[this] { m_vision.ToggleTargetingMode(); }, {&m_vision} };
    frc2::InstantCommand m_turretToCenter{[this] { m_turret.TurnTo(0.0); }, {&m_turret} };
    frc2::InstantCommand m_turretToPosStop{[this] { m_turret.TurnTo(TurretConstants::kMaxAngle); }, {&m_turret} };
    frc2::InstantCommand m_turretToNegStop{[this] { m_turret.TurnTo(TurretConstants::kMinAngle); }, {&m_turret} };
    frc2::InstantCommand m_runCompressor{[this] { m_compressor.EnableDigital(); m_bRunningCompressor = true;}, {} };
    // frc2::InstantCommand m_resetOdoAndGyro{[this] 
    // { 
    //     m_gyro.SetHeading((double)trajectory.InitialPose().Rotation().Degrees()); 
    //     m_drive.ResetOdometry(trajectory.InitialPose());
    // }, {&m_drive}
    // };
    frc2::InstantCommand m_driveRotateCw
    { [this]
        { m_drive.Drive(units::meters_per_second_t(0.0), units::meters_per_second_t(0.0),units::angular_velocity::radians_per_second_t(0.1), m_fieldRelative); }, {&m_drive} 
    };
    frc2::InstantCommand m_driveRotateCcw
    { [this]
        { m_drive.Drive(units::meters_per_second_t(0.0), units::meters_per_second_t(0.0),units::angular_velocity::radians_per_second_t(-0.1), m_fieldRelative); }, {&m_drive} 
    };
    frc2::InstantCommand m_runTransferAndFeeder
    { [this]
        { 
            m_transfer.SetFeeder(0.5);
            m_transfer.SetTransfer(0.5);
        },
        {&m_transfer}
    };
    frc2::InstantCommand m_stopTransferAndFeeder
    { [this]
        { 
            m_transfer.SetFeeder(0.0);
            m_transfer.SetTransfer(0.0);
        },
        {&m_transfer}
    };
    frc2::InstantCommand m_testServoIfFlagSet
    { [this]
        { 
            if (m_dbgSeroTest)
            {
                auto s = SmartDashboard::GetNumber("servo override", 0.0);
                m_hood.SetServoPosition(s);
            }
        },
        {&m_hood}
    };
    frc2::InstantCommand m_setOneBallFlag{[this] { m_onlyOneBall = true; }, {} };
    frc2::InstantCommand m_resetOneBallFlag{[this] { m_onlyOneBall = false; }, {} };
    
    bool m_onlyOneBall = false;    // Used in auto to shoot one ball
    //bool m_hasAutoRun = false;
    bool m_turretready = false;
    bool m_firing = false;
    bool m_finished = false;
    bool m_bLowSpeedDriving = false;
    bool m_bRunningCompressor = false;

    radians_per_second_t m_maxRotSpeed { kDriveAngularSpeed };

    DebugFlag   m_dbgSeroTest{"ServoTest", false};
    DebugFlag   m_dbgContinousFlywheel{"ContinousFlywheel", true};
};
