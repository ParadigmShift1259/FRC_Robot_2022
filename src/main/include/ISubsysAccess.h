#pragma once

#include <frc/trajectory/Trajectory.h>

#include <units/time.h>

//#include "subsystems/DriveSubsystem.h"
//#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/TransferSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class ISubsysAccess
{
public:
    //virtual DriveSubsystem&      GetDrive() = 0;
    //virtual FlywheelSubsystem&   GetFlywheel() = 0;
    virtual HoodSubsystem&       GetHood() = 0;
    virtual IntakeSubsystem&     GetIntake() = 0;
    virtual TransferSubsystem&   GetTransfer() = 0;
    virtual TurretSubsystem&     GetTurret() = 0;
    virtual VisionSubsystem&     GetVision() = 0;

    virtual Pose2d GetPose() = 0;
    virtual Pose2d GetPose(units::time::second_t timestamp) const = 0;
    virtual const vector<frc::Trajectory::State>& GetStateHist() const = 0;
};
