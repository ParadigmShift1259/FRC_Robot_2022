#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/Timer.h>

#include "subsystems/TransportSubsystem.h"

class TransferFire : public frc2::CommandHelper<frc2::CommandBase, TransferFire> {
public:
    TransferFire(TransportSubsystem* transfer, 
                bool* turretready, bool* firing, bool* finished,
                double launctime);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;


 private:
    TransportSubsystem* m_transport;
    frc::Timer m_timer;
    bool* m_turretready;
    bool* m_firing;
    bool* m_finished;
    double m_launchtime;
};