#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TransferSubsystem.h"

#include "Constants.h"

class IntakeTransfer : public frc2::CommandHelper<frc2::SequentialCommandGroup, IntakeTransfer> {
public:
    IntakeTransfer(IntakeSubsystem* intake, TransferSubsystem* transfer, double speed);
};