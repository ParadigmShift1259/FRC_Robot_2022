#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/TransferSubsystem.h"

#include "commands/TransferPrepare.h"

class TransferLaunch : public frc2::CommandHelper<frc2::CommandBase, TransferLaunch> {
public:
    explicit TransferLaunch(TransferSubsystem* subsystem, 
                            bool* turretready, bool* firing, bool* finished,
                            double launchtime);

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
};