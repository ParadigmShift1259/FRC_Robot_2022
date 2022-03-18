// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>

void Robot::RobotInit() {}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
  //m_container.CloseLogFile();
  //Shuffleboard::StopRecording();
  m_container.TurretSetZeroAngle();
  m_container.GetDrive().m_enabled = false;
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  m_container.TurretSetZeroAngle();
  m_hasAutoRun = true;

  frc::DataLogManager::LogNetworkTables(false);
  frc::DataLogManager::Start();
  // Record both DS control and joystick data
  DriverStation::StartDataLog(DataLogManager::GetLog());

  //m_container.GyroSetZeroHeading();

  m_autonomousCommand = m_container.GetAutonomousCommand(m_container.m_chooser.GetSelected());

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic()
{
  m_container.GetDrive().m_enabled = true;
}

void Robot::TeleopInit()
{
  if (m_hasAutoRun == false)
  {
    frc::DataLogManager::Start();
  }

  //Shuffleboard::SetRecordingFileNameFormat("Team1259NetTblData${date}_${time}");
  //Shuffleboard::StartRecording();

//  if (m_container.HasAutoRun() == false)
     {
    // Test code -- NORMALLY THIS SHOULD BE SET AT t=0 OF AUTO
    m_container.TurretSetZeroAngle();  
    m_container.GyroSetZeroHeading();  
    m_container.ResetOdometry(frc::Pose2d(kFieldLength/2 - 132_in, kFieldWidth/2, Rotation2d{180_deg})); // test code: robot center 11 feet directly in front of hub
    printf("Resetting Odometry from Teleop: x=%.3f, y=%.3f, heading =%.1f\n", m_container.GetPose().X().to<double>(), m_container.GetPose().Y().to<double>(), m_container.GetPose().Rotation().Degrees().to<double>());
    
SmartDashboard::PutNumber("Hood Servo Pos Command", HoodConstants::kMin);
SmartDashboard::PutNumber("Flywheel RPM Command", 0.0);
SmartDashboard::PutNumber("Cam Pitch Angle", 22.0);
SmartDashboard::PutNumber("Cam Height", 38.0);

    }   
 
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

  m_container.ZeroDrive();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic()
{
  m_container.GetDrive().m_enabled = true;
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
