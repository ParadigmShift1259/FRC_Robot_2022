#pragma once

#include <frc/XboxController.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TransferSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "commands/TransferFire.h"
#include "commands/HomeTarget.h"

#include "Constants.h"

class Fire : public frc2::CommandHelper<frc2::ParallelCommandGroup, Fire> {
public:
    Fire(   frc::XboxController* controller, FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood,
            TransferSubsystem* transfer,
            bool* m_turretready, bool* m_firing, bool* m_finished, double distance,
            double launchtime = TransferConstants::kTimeLaunch);
private:
    frc::XboxController* m_controller;
    bool* m_turretready;
    bool* m_firing;
    bool* m_finished;
};