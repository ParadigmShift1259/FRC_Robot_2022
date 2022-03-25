#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/Timer.h>

#define SAVE
#ifdef SAVE
#include "subsystems/TransferSubsystem.h"
#else
#include "../ISubsysAccess.h"
#endif

class FireOneBall : public frc2::CommandHelper<frc2::CommandBase, FireOneBall> {
public:
#ifdef SAVE
    FireOneBall(TransferSubsystem* transfer);
#else
    FireOneBall(ISubsysAccess& subsystemAccess);
#endif

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

 private:
#ifdef SAVE
    TransferSubsystem* m_transfer;
#else
    ISubsysAccess& m_subsystemAccess;
#endif
    frc::Timer m_timer;
};