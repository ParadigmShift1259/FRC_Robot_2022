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
#include "common/Gyro.h"
#include "common/SwerveControllerCommand2.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"

#include <iostream>
// #include <wpi/Path.h>
#include <wpi/SmallString.h>

#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

class RobotContainer
{
public:
    RobotContainer();

    void Periodic();

    void ZeroDrive();
    enum AutoPath {kEx1, kEx2, kEx3, kEx4, kEx5};
    frc2::Command *GetAutonomousCommand(AutoPath path);

    frc::SendableChooser<AutoPath> m_chooser;
    
private:
    void SetDefaultCommands();
    void ConfigureButtonBindings();
    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> GetSwerveCommandPath(string pathName, bool primaryPath);
    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> GetSwerveCommand(double path[][6], int length, bool primaryPath);
    frc::Trajectory convertArrayToTrajectory(double a[][6], int length);
    frc::Trajectory convertPathToTrajectory(PathPlannerTrajectory path);
    void PrintTrajectory(frc::Trajectory& trajectory);

    frc::XboxController m_primaryController{OIConstants::kPrimaryControllerPort};
    frc::XboxController m_secondaryController{OIConstants::kSecondaryControllerPort};

    Gyro m_gyro;
    DriveSubsystem m_drive;
    bool m_fieldRelative = true;
    VisionSubsystem m_vision; 
    FlywheelSubsystem m_flywheel;
};
