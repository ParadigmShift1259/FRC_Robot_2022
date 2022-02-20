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
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc/geometry/Translation2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <wpi/fs.h>

#include "common/Util.h"
#include "Gyro.h"
#include "common/SwerveControllerCommand2.h"

#include "ISubsysAccess.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"

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

class RobotContainer : public ISubsysAccess
{
public:
    RobotContainer();

    void Periodic();

    void ZeroDrive();
    void TurretSetZeroAngle() { m_turret.SetZeroAngle(); }

    enum AutoPath {kEx1, kEx2, kEx3, kEx4, kEx5};
    frc2::Command *GetAutonomousCommand(AutoPath path);

    frc::SendableChooser<AutoPath> m_chooser;

    //DriveSubsystem&      GetDrive() override { return m_drive; };
    //FlywheelSubsystem&   GetFlywheel() override { return m_flywheel; };
    HoodSubsystem&       GetHood() override { return m_hood; };
    IntakeSubsystem&     GetIntake() override { return m_intake; };
    TransferSubsystem&   GetTransfer() override { return m_transfer; };
    TurretSubsystem&     GetTurret() override { return m_turret; };
    VisionSubsystem&     GetVision() override { return m_vision; };    
private:
    void SetDefaultCommands();
    void ConfigureButtonBindings();
    SwerveCtrlCmd GetSwerveCommandPath(string pathName, bool primaryPath);
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

    frc2::InstantCommand m_setFieldRelative{[this] { m_fieldRelative = true; }, {}};
    frc2::InstantCommand m_clearFieldRelative{[this] { m_fieldRelative = false; }, {}};
    frc2::InstantCommand m_zeroHeading{[this] { m_gyro.ZeroHeading(); }, {}};
    
    double m_overrideAngle = 0.0;

    bool m_turretready = false;
    bool m_firing = false;
    bool m_finished = false;
};
