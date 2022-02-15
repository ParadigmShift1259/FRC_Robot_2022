#include "commands/Unjam.h"
#include "Constants.h"

using namespace TransferConstants;
using namespace IntakeConstants;

Unjam::Unjam(TransportSubsystem* transport, IntakeSubsystem* intake)
 : m_transport(transport)
 , m_intake(intake)
{
    AddRequirements({transport, intake});
}

void Unjam::Execute() {
    m_transport->SetTransfer(-1.0 * kTransferSpeedIntaking);
    m_transport->SetFeeder(-1.0 * kFeederSpeed);
    m_intake->Set(kReleaseHigh);
}