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
constexpr kilogram_t flywheelMass = pound_t(6.0);
//?constexpr kilogram_t flywheelMass = pound_t(4.7);

constexpr meter_t flywheelRadius = 2.0_in;
constexpr scalar_t flywheelRotInertiaFrac = 1.0 / 2.0;
constexpr auto flywheelRotInertia = flywheelRotInertiaFrac * flywheelMass * flywheelRadius * flywheelRadius;

constexpr kilogram_t cargoMass = ounce_t(9.5);
constexpr meter_t cargoRadius = inch_t(4.75);
constexpr scalar_t cargoRotInertiaFrac = 2.0 / 3.0;
constexpr auto cargoRotInertia = cargoRotInertiaFrac * cargoMass * cargoRadius * cargoRadius;

constexpr auto massRatio = flywheelMass / cargoMass;
constexpr auto rotInertiaRatio = flywheelRotInertia / cargoRotInertia;

constexpr dimensionless_t linearRegSlope = 0.9;
//constexpr dimensionless_t linearRegSlope = 0.874;
constexpr auto linearRegConst = radians_per_second_t (12.7);

constexpr degree_t maxAngle = degree_t(50.0);
constexpr degree_t minAngle = degree_t(23.0);

constexpr foot_t robotHeight = foot_t(3.5);
constexpr foot_t defaultTargetDist = foot_t(2.0);
constexpr foot_t defaultTargetHeight = foot_t(8.67);
constexpr foot_t defaultHeightAboveHub = foot_t(9.0);

class Calculations
{
 public:
  Calculations();

  meter_t HubHeightToMaxHeight();
  second_t CalcTimeOne();
  second_t CalcTimeTwo();
  second_t CalcTotalTime();
  meters_per_second_t CalcInitXVel();
  meters_per_second_t CalcInitYVel();
  meters_per_second_t CalcInitVel();
  meters_per_second_t CalcInitVelWithAngle();
  degree_t GetInitAngle();                                                            //!< Call after GetInitVelWithAngle or GetInitRPMS
  revolutions_per_minute_t CalcInitRPMs(meter_t distance, meter_t targetDist, meter_t targetHeight = defaultTargetHeight, meter_t heightAboveHub = defaultHeightAboveHub);        //!< Calculates the RPMs needed to shoot the specified distance
  radians_per_second_t QuadraticFormula(double a, double b, double c, bool subtract);

  void CalculateAll();

 private:
  second_t m_timeOne = second_t(0.0);
  second_t m_timeTwo = second_t(0.0);
  second_t m_timeTotal = second_t(0.0);

  meter_t m_heightAboveHub = foot_t(defaultHeightAboveHub);
  meter_t m_heightRobot = foot_t(robotHeight);
  meter_t m_heightTarget = foot_t(defaultTargetHeight);
  meter_t m_heightMax = meter_t(16.0);

  meter_t m_xInput = meter_t(0.0);
  meter_t m_xTarget = foot_t(defaultTargetDist);

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
