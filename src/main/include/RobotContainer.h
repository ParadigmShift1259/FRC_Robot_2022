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

#include "common/Util.h"
#include "Gyro.h"
#include "common/SwerveControllerCommand2.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"

#include "commands/CyclerAgitation.h"
#include "commands/CyclerIntakeAgitation.h"
#include "commands/IntakeIngest.h"
#include "commands/Unjam.h"
#include "commands/IntakeRelease.h"
#include "commands/Fire.h"

#include "Constants.h"

#include <iostream>
// #include <wpi/Path.h>
#include <wpi/SmallString.h>

class RobotContainer
{
public:
    RobotContainer();

    void Periodic();

    void ZeroDrive();
    enum AutoPath {kEx1, kEx2, kEx3, kEx4};
    frc2::Command *GetAutonomousCommand(AutoPath path);

    frc::SendableChooser<AutoPath> m_chooser;
    
private:
    void SetDefaultCommands();
    void ConfigureButtonBindings();
    frc2::SwerveControllerCommand2<DriveConstants::kNumSwerveModules> GetSwerveCommand(double path[][6], int length, bool primaryPath);
    frc::Trajectory convertArrayToTrajectory(double a[][6], int length);
    void PrintTrajectory(frc::Trajectory& trajectory);

    frc::XboxController m_primaryController{OIConstants::kPrimaryControllerPort};
    frc::XboxController m_secondaryController{OIConstants::kSecondaryControllerPort};

    Gyro2 m_gyro;
    DriveSubsystem m_drive;
    bool m_fieldRelative = true;
    VisionSubsystem m_vision; 
    FlywheelSubsystem m_flywheel;
    IntakeSubsystem m_intake;
    CyclerSubsystem m_cycler;
    TurretSubsystem m_turret = TurretSubsystem(&m_gyro);
    HoodSubsystem m_hood;

    //bool m_fieldRelative = true;
    bool m_turretready = false;
    bool m_firing = false;
    bool m_finished = false;
};
