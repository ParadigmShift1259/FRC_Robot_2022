// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//#pragma once

#include <memory>
#include <utility>

#include "SwerveControllerCommand2.h"

namespace frc2 {

template <size_t NumModules>
SwerveControllerCommand2<NumModules>::SwerveControllerCommand2(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<frc::Rotation2d()> desiredRotation,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::initializer_list<Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_desiredRotation(std::move(desiredRotation)),
      m_outputStates(output)
{
    this->AddRequirements(requirements);
}

template <size_t NumModules>
SwerveControllerCommand2<NumModules>::SwerveControllerCommand2(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::initializer_list<Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_outputStates(output)
{
  this->AddRequirements(requirements);
  m_desiredRotation = [&] {
    return m_trajectory.States().back().pose.Rotation();
  };
}

template <size_t NumModules>
SwerveControllerCommand2<NumModules>::SwerveControllerCommand2(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<frc::Rotation2d()> desiredRotation,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    wpi::span<Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_desiredRotation(std::move(desiredRotation)),
      m_outputStates(output)
{
  this->AddRequirements(requirements);
}

template <size_t NumModules>
SwerveControllerCommand2<NumModules>::SwerveControllerCommand2(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    wpi::span<Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_outputStates(output)
{
    this->AddRequirements(requirements);
    m_desiredRotation = [&] {
      return m_trajectory.States().back().pose.Rotation();
    };
}

template <size_t NumModules>
void SwerveControllerCommand2<NumModules>::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

template <size_t NumModules>
void SwerveControllerCommand2<NumModules>::Execute() {
  auto curTime = units::second_t(m_timer.Get());
  auto m_desiredState = m_trajectory.Sample(curTime);

  //auto targetChassisSpeeds = m_controller.Calculate(m_pose(), m_desiredState, m_desiredRotation());
  auto targetChassisSpeeds = m_controller.Calculate(m_pose(), m_desiredState, m_desiredState.pose.Rotation());
  printf("t=%.3f desired x=%.3f y=%.3f Rot=%.3f ", curTime.to<double>(), m_desiredState.pose.X().to<double>(), m_desiredState.pose.Y().to<double>(), m_desiredState.pose.Rotation().Degrees().to<double>());
  //printf("odometer x %.3f y %.3f Rot %.3f\n", m_pose().X().to<double>(), m_pose().Y().to<double>(), m_pose().Rotation().Degrees().to<double>());
  double err_x = m_desiredState.pose.X().to<double>() - m_pose().X().to<double>();
  double err_y = m_desiredState.pose.Y().to<double>() - m_pose().Y().to<double>();
  double err_theta = m_desiredState.pose.Rotation().Degrees().to<double>() - m_pose().Rotation().Degrees().to<double>();
  printf("speed_cmd=%.3f, x err=%.3f, y err=%.3f, theta err=%.3f\n", m_desiredState.velocity.to<double>(), err_x, err_y, err_theta);

  auto targetModuleStates = m_kinematics.ToSwerveModuleStates(targetChassisSpeeds);

  m_outputStates(targetModuleStates);
}

template <size_t NumModules>
void SwerveControllerCommand2<NumModules>::End(bool interrupted) {
  m_timer.Stop();
}

template <size_t NumModules>
bool SwerveControllerCommand2<NumModules>::IsFinished() {
  return m_timer.HasElapsed(m_trajectory.TotalTime());
}

}  // namespace frc2