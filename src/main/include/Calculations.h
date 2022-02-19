/// Physics/Ballistics calculations for FRC 2022 Game RapidReact

#pragma once

#include <units/time.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <units/moment_of_inertia.h>
#include <units/mass.h>
#include <units/energy.h>
#include <units/dimensionless.h>

#include <networktables/NetworkTableEntry.h>

using namespace units;

/// Ballistics/Physics constants
constexpr auto gravity = meters_per_second_squared_t(9.81);
constexpr kilogram_t flywheelMass = pound_t(5.0);
//?constexpr kilogram_t flywheelMass = pound_t(4.7);

constexpr meter_t flywheelRadius = 2.0_in;
constexpr auto flywheelRotInertia =  0.5 * flywheelMass * flywheelRadius * flywheelRadius;

constexpr kilogram_t cargoMass = ounce_t(9.5);
constexpr meter_t cargoRadius = inch_t(4.75);
constexpr auto cargoRotInertia = 2.0 / 3.0 * cargoMass * cargoRadius * cargoRadius;

constexpr auto rotInertiaRatio = flywheelRotInertia / cargoRotInertia;

constexpr dimensionless_t linearRegSlope = 0.874;
constexpr auto linearRegConst = radians_per_second_t (12.7);

// auto test = flywheelRotInertia * radians_per_second_squared_t(10.0);
// auto test = flywheelRotInertia * radians_per_second_squared_t(10.0) / radian_t(10.0);

class Calculations
{
 public:
  Calculations();

  auto HubHeightToMaxHeight();
  auto GetTimeOne();
  auto GetTimeTwo();
  auto GetTotalTime();
  auto GetInitXVel();
  auto GetInitYVel();
  auto GetInitVelWithAngle();
  degree_t GetInitAngle();                //!< Call after GetInitVelWithAngle or GetInitRPMS
  revolutions_per_minute_t GetInitRPMS(meter_t distance);
  radians_per_second_t QuadraticFormula(double a, double b, double c, bool subtract);

  void CalculateAll();

 private:
  second_t m_timeOne = second_t(0.0);
  second_t m_timeTwo = second_t(0.0);
  second_t m_timeTotal = second_t(0.0);

  meter_t m_heightAboveHub = meter_t(0.0);
  meter_t m_heightRobot = meter_t(0.0);
  meter_t m_heightTarget = meter_t(0.0);
  meter_t m_heightMax = meter_t(0.0);

  meter_t m_xInput = meter_t(0.0);
  meter_t m_xTarget = meter_t(0.0);

  meters_per_second_t m_velXInit = meters_per_second_t (0.0);
  meters_per_second_t m_velYInit = meters_per_second_t(0.0);
  meters_per_second_t m_velInit = meters_per_second_t(0.0);

  degree_t m_angleInit = degree_t(0.0);

  radians_per_second_t m_rotVelInit = radians_per_second_t(0.0);
  revolutions_per_minute_t m_rpmInit = revolutions_per_minute_t(0.0);

  nt::NetworkTableEntry m_heightAboveHubEntry;
  nt::NetworkTableEntry m_heightRobotEntry;
  nt::NetworkTableEntry m_heightTargetEntry;
  nt::NetworkTableEntry m_xFloorDistanceEntry;
  nt::NetworkTableEntry m_xTargetDistanceEntry;

  nt::NetworkTableEntry m_initVelEntry;
  nt::NetworkTableEntry m_initAngleEntry;
  nt::NetworkTableEntry m_initRpmEntry;
  nt::NetworkTableEntry m_setpointEntry;

};
