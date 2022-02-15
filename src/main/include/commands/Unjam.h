#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TransportSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

class Unjam : public frc2::CommandHelper<frc2::CommandBase, Unjam> {
public:
    explicit Unjam(TransportSubsystem* transport, IntakeSubsystem* intake);

    void Execute() override;

 private:
    TransportSubsystem* m_transport;
    IntakeSubsystem* m_intake;
};