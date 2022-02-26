#include "commands/Unjam.h"
#include "Constants.h"

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

void Unjam::End(bool bInterrupted)
{
    m_transfer->SetTransfer(0.0);
    m_transfer->SetFeeder(0.);
    m_intake->Set(0.0);
}
