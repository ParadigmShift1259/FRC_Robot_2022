#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TransferSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class Unjam : public frc2::CommandHelper<frc2::CommandBase, Unjam> {
public:
    explicit Unjam(TransferSubsystem* transfer, IntakeSubsystem* intake);

    void Execute() override;

 private:
    TransferSubsystem* m_transfer;
    IntakeSubsystem* m_intake;
};