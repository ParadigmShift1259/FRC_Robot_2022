#pragma once

#include <frc/XboxController.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "Calculations.h"

class HomeTarget : public frc2::CommandHelper<frc2::CommandBase, HomeTarget> {
public:
    explicit HomeTarget(  frc::XboxController* controller
                        , FlywheelSubsystem* flywheel
                        , TurretSubsystem* turret
                        , HoodSubsystem* hood
                        , bool* turretready
                        , bool* firing
                        , bool* finished);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    frc::XboxController* m_controller;
    FlywheelSubsystem* m_flywheel;
    TurretSubsystem* m_turret;
    HoodSubsystem* m_hood;
    bool* m_turretready;
    bool* m_firing;
    bool* m_finished;
  	//Calculations m_calculation;
};