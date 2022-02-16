#include "commands/TransferFirstBall.h"
#include "commands/IntakeTransfer.h"
#include "Constants.h"

using namespace TransferConstants;

TransferFirstBall::TransferFirstBall(TransportSubsystem* transport)
 : m_transport(transport)
 {
  AddRequirements({transport});
}

void TransferFirstBall::Initialize()
{
}

void TransferFirstBall::Execute()
{
    m_transport->SetTransfer(kTransferSpeedIntaking);
    m_transport->SetFeeder(kFeederSpeed);
}

bool TransferFirstBall::IsFinished() {
    return m_transport->GetFeederPhotoeye();
}

void TransferFirstBall::End(bool interrupted)
{
    m_transport->SetFeeder(0);
    m_transport->SetTransfer(0);
}