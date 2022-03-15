#pragma once

#include <units/angle.h>
#include <frc/trajectory/Trajectory.h>

using namespace units;

class StateWithTurretAngle : public frc::Trajectory::State
{
public:
    degree_t m_turretAngle = 0_deg;

    /// Checks equality between this State and another object.
    ///
    /// @param other The other object.
    /// @return Whether the two objects are equal.
    bool operator==(const StateWithTurretAngle& other) const
    {
      return   t == other.t 
            && velocity == other.velocity 
            && acceleration == other.acceleration 
            && pose == other.pose 
            && curvature == other.curvature
            && m_turretAngle == other.m_turretAngle;
    }

    /// Checks inequality between this State and another object.
    ///
    /// @param other The other object.
    /// @return Whether the two objects are not equal.
    bool operator!=(const StateWithTurretAngle& other) const
    {
        return !operator==(other);
    }
};
