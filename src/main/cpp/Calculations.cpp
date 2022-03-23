#include "Calculations.h"

#include "Constants.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <wpi/StringMap.h>
#include <stdio.h>
#include <units/energy.h>

using namespace units;

Calculations::Calculations()
{
    wpi::StringMap<std::shared_ptr<nt::Value>> propMap0_10(3);
    wpi::StringMap<std::shared_ptr<nt::Value>> propMap0_4(3);
    //wpi::StringMap<std::shared_ptr<nt::Value>> propMap0_25(3);

    propMap0_10.insert(std::make_pair("Min", nt::Value::MakeDouble(0.0)));
    propMap0_10.insert(std::make_pair("Max", nt::Value::MakeDouble(16.0)));
    propMap0_10.insert(std::make_pair("Block increment", nt::Value::MakeDouble(1.0 / 100.0)));

    propMap0_4.insert(std::make_pair("Min", nt::Value::MakeDouble(0.0)));
    propMap0_4.insert(std::make_pair("Max", nt::Value::MakeDouble(4.0)));
    propMap0_4.insert(std::make_pair("Block increment", nt::Value::MakeDouble(1.0 / 10.0)));

    // propMap0_25.insert(std::make_pair("Min", nt::Value::MakeDouble(0.0)));
    // propMap0_25.insert(std::make_pair("Max", nt::Value::MakeDouble(25.0)));
    // propMap0_25.insert(std::make_pair("Block increment", nt::Value::MakeDouble(1.0)));

    frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Calculations");
    
    int xPos = 0;
    m_xTargetDistanceEntry = tab.Add("TargetDistance", defaultTargetDist.to<double>())
                                .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                                .WithSize(1, 1)
                                .WithPosition(xPos++, 0)
                                .WithProperties(propMap0_4)
                                .GetEntry();

    m_heightTargetEntry = tab.Add("TargetHeight", defaultTargetHeight.to<double>())
                            .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                            .WithSize(1, 1)
                            .WithPosition(xPos++, 0)
                            .WithProperties(propMap0_10)
                            .GetEntry();

    m_heightAboveHubEntry = tab.Add("HeightAboveHub", defaultHeightAboveHub.to<double>())
                              .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                              .WithSize(1, 1)
                              .WithPosition(xPos++, 0)
                              .WithProperties(propMap0_10)
                              .GetEntry();

    // m_heightRobotEntry = tab.Add("RobotHeight", robotHeight.to<double>())
    //                         .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    //                         .WithSize(1, 1)
    //                         .WithPosition(xPos++, 0)
    //                         .WithProperties(propMap0_10)
    //                         .GetEntry();

    // m_xFloorDistanceEntry = tab.Add("FloorDistance", 6.0)
    //                           .WithWidget(frc::BuiltInWidgets::kNumberSlider)
    //                           .WithSize(1, 1)
    //                           .WithPosition(xPos++, 0)
    //                           .WithProperties(propMap0_25)
    //                           .GetEntry();

    xPos = 0;
    m_initVelEntry = tab.Add("Initial Velocity", 0.0)
                        .WithWidget(frc::BuiltInWidgets::kTextView)
                        .WithSize(1, 1)
                        .WithPosition(xPos++, 1)
                        .GetEntry();

    m_initAngleEntry = tab.Add("Inital Angle", 0.0)
                          .WithWidget(frc::BuiltInWidgets::kTextView)
                          .WithSize(1, 1)
                          .WithPosition(xPos++, 1)
                          .GetEntry();

    m_initRpmEntry = tab.Add("RPMs", 0.0)
                        .WithWidget(frc::BuiltInWidgets::kTextView)
                        .WithSize(1, 1)
                        .WithPosition(xPos++, 1)
                        .GetEntry();

    m_setpointEntry = tab.Add("SetPoint", 0.0)
                        .WithWidget(frc::BuiltInWidgets::kTextView)
                        .WithSize(1, 1)
                        .WithPosition(xPos++, 1)
                        .GetEntry();
}

meter_t Calculations::HubHeightToMaxHeight()
{
  auto aValue = (m_xInput * (m_heightTarget - m_heightRobot) - (m_xInput + m_xTarget) * (m_heightAboveHub - m_heightRobot)) / (m_xTarget * m_xInput * (m_xInput + m_xTarget));
  auto bValue = ((m_xInput + m_xTarget) * (m_xInput + m_xTarget) * (m_heightAboveHub - m_heightRobot) - m_xInput * m_xInput * (m_heightTarget - m_heightRobot)) / (m_xTarget * m_xInput * (m_xInput + m_xTarget));

  m_heightMax = -1.0 * bValue * bValue / (4.0 * aValue) + m_heightRobot;

  return m_heightMax;
}

second_t Calculations::CalcTimeOne()
{
  m_timeOne = math::sqrt(2.0 * (m_heightMax - m_heightRobot) / gravity);

  return m_timeOne;
}

second_t Calculations::CalcTimeTwo()
{
  m_timeTwo = math::sqrt(2.0 * (m_heightMax - m_heightTarget) / gravity);

  return m_timeTwo;
}

second_t Calculations::CalcTotalTime()
{
  m_timeTotal = CalcTimeOne() + CalcTimeTwo();

  return m_timeTotal;
}

meters_per_second_t Calculations::CalcInitXVel()
{
  m_velXInit = (m_xInput + m_xTarget) / CalcTotalTime();

  return m_velXInit;
}

meters_per_second_t Calculations::CalcInitYVel()
{
  m_velYInit = math::sqrt(2.0 * gravity * (m_heightMax - m_heightRobot));

  return m_velYInit;
}

meters_per_second_t Calculations::CalcInitVel()
{
  HubHeightToMaxHeight();

  CalcInitXVel();
  CalcInitYVel();
  
  m_angleInit = math::atan(m_velYInit / m_velXInit);
  m_angleInit = degree_t(std::clamp(m_angleInit.to<double>(), minAngle.to<double>(), maxAngle.to<double>()));

  CalcInitVelWithAngle();

  m_initVelEntry.SetDouble(m_velInit.to<double>());
  m_initAngleEntry.SetDouble(m_angleInit.to<double>());

  return m_velInit;
}

meters_per_second_t Calculations::CalcInitVelWithAngle() {
  meter_t totalXDist = m_xInput + m_xTarget;
  meter_t totalYDist = m_heightTarget - m_heightRobot;

  m_velInit = math::sqrt(gravity * totalXDist * totalXDist / (2.0 * (totalXDist * math::tan(m_angleInit) - totalYDist))) / math::cos(m_angleInit);
  return m_velInit;
}

degree_t Calculations::GetInitAngle()
{
  return m_angleInit;
}

revolutions_per_minute_t Calculations::CalcInitRPMs(meter_t distance, meter_t targetDist, meter_t heightAboveHub/* = defaultHeightAboveHub*/, meter_t targetHeight/* = defaultTargetHeight*/)
{
  m_xInput = distance;
  m_xTarget = targetDist;
  m_heightTarget = targetHeight;
  m_heightAboveHub = heightAboveHub;

#define TRAJECTORY_TUNING
#ifdef TRAJECTORY_TUNING
    m_xTarget = foot_t(m_xTargetDistanceEntry.GetDouble(defaultTargetDist.to<double>()));
    m_heightTarget = foot_t(m_heightTargetEntry.GetDouble(defaultTargetHeight.to<double>()));
    //m_heightAboveHub = foot_t(m_heightAboveHubEntry.GetDouble(defaultHeightAboveHub.to<double>()));
    m_heightAboveHubEntry.SetDouble(m_heightAboveHub.convert<foot>().to<double>());
#else
    m_xTargetDistanceEntry.SetDouble(m_xTarget.to<double>());
    m_heightTargetEntry.SetDouble(m_heightTarget.to<double>());
    m_heightAboveHubEntry.SetDouble(m_heightAboveHub.to<double>());
#endif
  
  CalcInitVel();

  m_rotVelInit = radian_t(1.0) * m_velInit / flywheelRadius * (2.0 + (cargoRotInertiaFrac + 1.0) / (flywheelRotInertiaFrac * massRatio));
  m_rpmInit = m_rotVelInit;

  m_initRpmEntry.SetDouble(m_rpmInit.to<double>());
  m_setpointEntry.SetDouble(m_rpmInit.to<double>() / FlywheelConstants::kGearRatio);

  return m_rpmInit;
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

void Calculations::CalculateAll()
{
  //printf("Calculate All Called\n");

  FILE *calcFile = fopen("/tmp/calcfile.txt", "w");

  fprintf(calcFile, "HeightAboveHub, TargetHeight, RobotHeight, FloorDist, TargetDist, RPMs, Setpoint, InitialVelocity, InitialAngle\n");

  for (double i=8.0; i<25.0; i++)
  {
    for (double j=0.1; j<4.0; j+=0.1)
    {
      for (double k=9.0; k<10.0; k+=0.1)
      {
        CalcInitRPMs(foot_t(i), foot_t(j), foot_t(8.67), foot_t(k));

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
  }

  fclose(calcFile);
}
