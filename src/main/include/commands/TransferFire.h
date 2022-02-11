#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/TransferSubsystem.h"

#include "commands/TransferLaunch.h"
#include "commands/TransferPrepare.h"
///#include "commands/HomeTarget.h"

///#include "Constants.h"

class TransferFire : public frc2::CommandHelper<frc2::SequentialCommandGroup, TransferFire> {
public:
    TransferFire(TransferSubsystem* transfer, 
                bool* turretready, bool* firing, bool* finished,
                double launctime);
};