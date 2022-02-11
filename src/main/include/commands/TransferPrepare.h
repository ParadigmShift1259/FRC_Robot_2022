#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TransferSubsystem.h"

class TransferPrepare : public frc2::CommandHelper<frc2::CommandBase, TransferPrepare>
{
public:
    explicit TransferPrepare(TransferSubsystem* subsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

 private:
    TransferSubsystem* m_transfer;
    bool reset;
};