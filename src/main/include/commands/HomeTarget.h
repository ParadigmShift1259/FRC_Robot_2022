#pragma once

#include <frc/XboxController.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#include "Calculations.h"

#include <functional>

using GetYvelocityCallback = std::function<double()>;

class HomeTarget : public frc2::CommandHelper<frc2::CommandBase, HomeTarget> {
public:
    explicit HomeTarget(  FlywheelSubsystem* flywheel
                        , TurretSubsystem* turret
                        , HoodSubsystem* hood
                        , VisionSubsystem& vision
                        , bool* turretready
                        , bool* firing
                        , bool* finished
                        , GetYvelocityCallback yVelocityCb);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    FlywheelSubsystem* m_flywheel;
    TurretSubsystem* m_turret;
    HoodSubsystem* m_hood;
    VisionSubsystem& m_vision;
    bool* m_turretready;
    bool* m_firing;
    bool* m_finished;
    GetYvelocityCallback m_yVelocityCb;
  	Calculations m_calculation;

};