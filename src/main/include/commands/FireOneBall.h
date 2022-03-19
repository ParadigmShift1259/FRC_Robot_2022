#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/Timer.h>

#include "subsystems/TransferSubsystem.h"

class FireOneBall : public frc2::CommandHelper<frc2::CommandBase, FireOneBall> {
public:
    FireOneBall(TransferSubsystem* transfer);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;


 private:
    TransferSubsystem* m_transfer;
    frc::Timer m_timer;
};