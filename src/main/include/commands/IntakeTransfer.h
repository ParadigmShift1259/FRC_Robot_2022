#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "ISubsysAccess.h"

class IntakeTransfer : public frc2::CommandHelper<frc2::SequentialCommandGroup, IntakeTransfer> {
public:
    IntakeTransfer(ISubsysAccess& subSysAccess, bool retractIntake);
};