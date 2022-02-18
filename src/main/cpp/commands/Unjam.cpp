#include "commands/Unjam.h"
#include "ConstantsIntake.h"
#include "ConstantsTransport.h"

using namespace TransferConstants;
using namespace IntakeConstants;

Unjam::Unjam(TransferSubsystem* transfer, IntakeSubsystem* intake)
 : m_transfer(transfer)
 , m_intake(intake)
{
    AddRequirements({transfer, intake});
}

void Unjam::Execute() {
    m_transfer->SetTransfer(-1.0 * kTransferSpeedIntaking);
    m_transfer->SetFeeder(-1.0 * kFeederSpeed);
    m_intake->Set(kReleaseHigh);
}