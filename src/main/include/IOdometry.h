#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <units/time.h>

#include <vector>

class IOdometry
{
public:
    virtual frc::Pose2d GetPose() = 0;
    virtual frc::Pose2d GetPose(units::time::second_t timestamp) const = 0;
    virtual const std::vector<frc::Trajectory::State>& GetStateHist() const = 0;
};
