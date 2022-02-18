#include "Calculations.h"

#include "Constants.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <wpi/StringMap.h>
#include <stdio.h>

using namespace units;

Calculations::Calculations() {

  wpi::StringMap<std::shared_ptr<nt::Value>> propMap0_10(3);
  wpi::StringMap<std::shared_ptr<nt::Value>> propMap0_4(3);
  wpi::StringMap<std::shared_ptr<nt::Value>> propMap0_25(3);

  propMap0_10.insert(std::make_pair("Min", nt::Value::MakeDouble(0.0)));
  propMap0_10.insert(std::make_pair("Max", nt::Value::MakeDouble(10.0)));
  propMap0_10.insert(std::make_pair("Block increment", nt::Value::MakeDouble(1.0 / 12.0)));

  propMap0_4.insert(std::make_pair("Min", nt::Value::MakeDouble(0.0)));
  propMap0_4.insert(std::make_pair("Max", nt::Value::MakeDouble(4.0)));
  propMap0_4.insert(std::make_pair("Block increment", nt::Value::MakeDouble(1.0 / 12.0)));

  propMap0_25.insert(std::make_pair("Min", nt::Value::MakeDouble(0.0)));
  propMap0_25.insert(std::make_pair("Max", nt::Value::MakeDouble(25.0)));
  propMap0_25.insert(std::make_pair("Block increment", nt::Value::MakeDouble(1.0)));

  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Calculations");
  
  m_heightAboveHubEntry = tab.Add("HeightAboveHub", 0.0)
                            .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                            .WithSize(1, 1)
                            .WithPosition(0, 0)
                            .WithProperties(propMap0_10)
                            .GetEntry();

  m_heightTargetEntry = tab.Add("TargetHeight", 0.0)
                          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                          .WithSize(1, 1)
                          .WithPosition(1, 0)
                          .WithProperties(propMap0_10)
                          .GetEntry();

  m_heightRobotEntry = tab.Add("RobotHeight", 0.0)
                          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                          .WithSize(1, 1)
                          .WithPosition(2, 0)
                          .WithProperties(propMap0_10)
                          .GetEntry();

  m_xFloorDistanceEntry = tab.Add("FloorDistance", 0.0)
                            .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                            .WithSize(1, 1)
                            .WithPosition(3, 0)
                            .WithProperties(propMap0_25)
                            .GetEntry();

  m_xTargetDistanceEntry = tab.Add("TargetDistance", 0.0)
                              .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                              .WithSize(1, 1)
                              .WithPosition(4, 0)
                              .WithProperties(propMap0_4)
                              .GetEntry();

  m_initVelEntry = tab.Add("Initial Velocity", 0.0)
                      .WithWidget(frc::BuiltInWidgets::kTextView)
                      .WithSize(1, 1)
                      .WithPosition(0, 1)
                      .GetEntry();

  m_initAngleEntry = tab.Add("Inital Angle", 0.0)
                        .WithWidget(frc::BuiltInWidgets::kTextView)
                        .WithSize(1, 1)
                        .WithPosition(1, 1)
                        .GetEntry();

  m_initRpmEntry = tab.Add("RPMs", 0.0)
                      .WithWidget(frc::BuiltInWidgets::kTextView)
                      .WithSize(1, 1)
                      .WithPosition(2, 1)
                      .GetEntry();

  m_setpointEntry = tab.Add("SetPoint", 0.0)
                      .WithWidget(frc::BuiltInWidgets::kTextView)
                      .WithSize(1, 1)
                      .WithPosition(3, 1)
                      .GetEntry();

  // frc::SmartDashboard::PutNumber("HeightAboveHub", m_heightAboveHub.to<double>());
  // frc::SmartDashboard::PutNumber("TargetHeight", m_heightTarget.to<double>());
  // frc::SmartDashboard::PutNumber("RobotHeight", m_heightRobot.to<double>());
  // frc::SmartDashboard::PutNumber("FloorHubDistance", m_xInput.to<double>());
  // frc::SmartDashboard::PutNumber("TargetXDistance", m_xTarget.to<double>());
}

auto Calculations::HubHeightToMaxHeight()
{
  auto aValue = (m_xInput * (m_heightTarget - m_heightRobot) - (m_xInput + m_xTarget) * (m_heightAboveHub - m_heightRobot)) / (m_xTarget * m_xInput * (m_xInput + m_xTarget));
  auto bValue = ((m_xInput + m_xTarget) * (m_xInput + m_xTarget) * (m_heightAboveHub - m_heightRobot) - m_xInput * m_xInput * (m_heightTarget - m_heightRobot)) / (m_xTarget * m_xInput * (m_xInput + m_xTarget));

  m_heightMax = -1.0 * bValue * bValue / (4.0 * aValue) + m_heightRobot;

  // frc::SmartDashboard::PutNumber("HubHeightToMaxHeight", units::foot_t(m_heightMax).to<double>());

  return m_heightMax;
}

auto Calculations::GetTimeOne()
{
  m_timeOne = math::sqrt(2.0 * (m_heightMax - m_heightRobot) / gravity);

  return m_timeOne;
}

auto Calculations::GetTimeTwo()
{
  m_timeTwo = math::sqrt(2.0 * (m_heightMax - m_heightTarget) / gravity);

  return m_timeTwo;
}

auto Calculations::GetTotalTime()
{
  m_timeTotal = GetTimeOne() + GetTimeTwo();

  return m_timeTotal;
}

auto Calculations::GetInitXVel()
{
  m_velXInit = (m_xInput + m_xTarget) / GetTotalTime();

  return m_velXInit;
}

auto Calculations::GetInitYVel()
{
  m_velYInit = math::sqrt(2.0 * gravity * (m_heightMax - m_heightRobot));

  return m_velYInit;
}

auto Calculations::GetInitVelWithAngle()
{
  // m_heightAboveHub = foot_t(frc::SmartDashboard::GetNumber("HeightAboveHub", 0.0));
  // m_heightTarget = foot_t(frc::SmartDashboard::GetNumber("TargetHeight", 0.0));
  // m_heightRobot = foot_t(frc::SmartDashboard::GetNumber("RobotHeight", 0.0));
  // m_xInput = foot_t(frc::SmartDashboard::GetNumber("FloorHubDistance", 0.0));
  // m_xTarget = foot_t(frc::SmartDashboard::GetNumber("TargetXDistance", 0.0));

  m_heightAboveHub = foot_t(m_heightAboveHubEntry.GetDouble(0.0));
  m_heightTarget = foot_t(m_heightTargetEntry.GetDouble(0.0));
  m_heightRobot = foot_t(m_heightRobotEntry.GetDouble(0.0));
  m_xInput = foot_t(m_xFloorDistanceEntry.GetDouble(0.0));
  m_xTarget = foot_t(m_xTargetDistanceEntry.GetDouble(0.0));
    
  m_heightMax = HubHeightToMaxHeight();

  GetInitXVel();
  GetInitYVel();
  
  m_velInit = math::hypot(m_velXInit, m_velYInit);
  m_angleInit = math::atan(m_velYInit / m_velXInit);

  // printf("InitVel %.3f\n", m_velInit.to<double>());
  // printf("InitAngle %.3f\n", m_angleInit.to<double>());

  m_initVelEntry.SetDouble(m_velInit.to<double>());
  m_initAngleEntry.SetDouble(m_angleInit.to<double>());

  // frc::SmartDashboard::PutNumber("InitVel", units::feet_per_second_t(m_velInit).to<double>());
  // frc::SmartDashboard::PutNumber("InitAngle", m_angleInit.to<double>());

  return m_velInit;
}

revolutions_per_minute_t Calculations::GetInitRPMS()
{
  GetInitVelWithAngle();

  auto aValue = flywheelRotInertia * (linearRegSlope - 1.0) * (linearRegSlope + linearRegSlope * rotInertiaRatio - rotInertiaRatio + 1);
  auto bValue = 2 * flywheelRotInertia * linearRegConst * (linearRegSlope + linearRegSlope * rotInertiaRatio - rotInertiaRatio);
  auto cValue = flywheelRotInertia * linearRegConst / radian_t (1.0) * linearRegConst / radian_t (1.0) * (rotInertiaRatio + 1.0) + cargoMass * m_velInit * m_velInit;
  
  m_rotVelInit = QuadraticFormula(aValue.to<double>(), bValue.to<double>(), cValue.to<double>(), (bool)true);
  m_rpmInit = m_rotVelInit;

  m_initRpmEntry.SetDouble(m_rpmInit.to<double>());
  m_setpointEntry.SetDouble(m_rpmInit.to<double>() / FlywheelConstants::kGearRatio);
  // frc::SmartDashboard::PutNumber("InitRPM", m_rpmInit.to<double>());

  return m_rotVelInit;
}

radians_per_second_t Calculations::QuadraticFormula(double a, double b, double c, bool subtract)
{
  auto outPut = radians_per_second_t(0.0);
  
  if (subtract == false)
    outPut = radians_per_second_t((-1.0 * b + sqrt(b * b - 4 * a * c)) / (2 * a));

  else
    outPut = radians_per_second_t((-1.0 * b - sqrt(b * b - 4 * a * c)) / (2 * a));

  return outPut;
}

void Calculations::CalculateAll() {
  FILE *calcFile = fopen("/tmp/calcfile.txt", "w");

  fprintf(calcFile, "HeightAboveHub, TargetHeight, RobotHeight, FloorDist, TargetDist, RPMs, Setpoint, InitialVelocity, InitialAngle\n");

  for (double i=8.0; i<25.0; i++) {
    for (double j=0.1; j<4; j+=0.1) {
      m_heightAboveHubEntry.SetDouble(9.0);
      m_heightTargetEntry.SetDouble(8.0 + 2.0/3.0);
      m_heightRobotEntry.SetDouble(3.0 + 5.0/6.0);
      m_xFloorDistanceEntry.SetDouble(i);
      m_xTargetDistanceEntry.SetDouble(j);

      GetInitRPMS();

      auto setpoint = m_rpmInit / FlywheelConstants::kGearRatio;

      fprintf(calcFile
              , "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n"
              , foot_t(m_heightAboveHub).to<double>()
              , foot_t(m_heightTarget).to<double>()
              , foot_t(m_heightRobot).to<double>()
              , foot_t(m_xInput).to<double>()
              , foot_t(m_xTarget).to<double>()
              , m_rpmInit.to<double>()
              , setpoint.to<double>()
              , feet_per_second_t(m_velInit).to<double>()
              , m_angleInit.to<double>());
    }
  }

  fclose(calcFile);
}