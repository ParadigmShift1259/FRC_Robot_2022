#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/Timer.h>

#include "subsystems/TransferSubsystem.h"

class TransferFire : public frc2::CommandHelper<frc2::CommandBase, TransferFire> {
public:
    TransferFire(TransferSubsystem* transfer, 
                bool* turretready, bool* firing, bool* finished,
                double launctime);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;


 private:
    TransferSubsystem* m_transfer;
    frc::Timer m_timer;
    bool* m_turretready;
    bool* m_firing;
    bool* m_finished;
    double m_launchtime;
    double m_startTime = 0.0;   // Temp
    bool m_oneBallFired;
};