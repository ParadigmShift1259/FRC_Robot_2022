#pragma once

#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/TransferSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class ISubsysAccess
{
public:
    virtual HoodSubsystem&       GetHood() = 0;
    virtual IntakeSubsystem&     GetIntake() = 0;
    virtual TransferSubsystem&   GetTransfer() = 0;
    virtual TurretSubsystem&     GetTurret() = 0;
    virtual VisionSubsystem&     GetVision() = 0;

    virtual bool OnlyOneBall() = 0;
};
