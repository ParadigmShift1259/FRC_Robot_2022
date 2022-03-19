#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <units/time.h>

#include "StateWithTurretAngle.h"

#include <vector>

//using StateHistColl = std::vector<frc::Trajectory::State>;
//using StateHist = frc::Trajectory::State;
using StateHist = StateWithTurretAngle;
using StateHistColl = std::vector<StateHist>;

class IOdometry
{
public:

    virtual frc::Pose2d GetPose() = 0;
    virtual frc::Pose2d GetPose(units::time::second_t timestamp) const = 0;
    virtual StateHist GetState() const = 0;
    virtual StateHist GetState(units::time::second_t timestamp) const = 0;
    virtual const StateHistColl& GetStateHist() const = 0;
    virtual units::degree_t GetTurretAngle() = 0;
    //virtual bool HasAutoRun() = 0;
    virtual bool OdoValid() = 0;
    virtual void ResetOdometry(frc::Pose2d) = 0;
};
