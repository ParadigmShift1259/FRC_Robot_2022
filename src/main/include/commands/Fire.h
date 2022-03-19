#pragma once

#include <frc/XboxController.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TransferSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "commands/FireOneBall.h"
// #include "commands/TransferFire.h"
#include "commands/TransferFirstBall.h"
#include "commands/HomeTarget.h"
#include "commands/WaitForFlywheel.h"

#include "Constants.h"

class Fire : public frc2::CommandHelper<frc2::SequentialCommandGroup, Fire> {
public:
    Fire( FlywheelSubsystem* flywheel
        , TurretSubsystem* turret
        , HoodSubsystem* hood
        , TransferSubsystem* transfer
        , VisionSubsystem& m_vision
        , bool* m_turretready
        , bool* m_firing
        , bool* m_finished
        , GetYvelocityCallback yVelocityCb
        , double launchtime = TransferConstants::kTimeLaunch);

    //void Execute() override;
    //void End(bool interrupted) override;

private:
    frc::XboxController* m_controller;
    FlywheelSubsystem* m_flywheel;
    bool* m_turretready;
    bool* m_firing;
    bool* m_finished;
};