/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc/trajectory/Trajectory.h>

//#include <frc2/command/SwerveControllerCommand.h>
#include "common/SwerveControllerCommand2.h"
#include "Gyro.h"
#include "ISubsysAccess.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"

#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;
using SwerveCtrlCmd = frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules>;

class RobotContainer : public ISubsysAccess
{
public:
    RobotContainer();

    void Periodic();

    void ZeroDrive();
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

    bool m_turretready = false;
    bool m_firing = false;
    bool m_finished = false;
};
