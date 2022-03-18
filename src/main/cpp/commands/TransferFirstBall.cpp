#include "commands/TransferFirstBall.h"
#include "commands/IntakeTransfer.h"
#include "Constants.h"

using namespace IntakeConstants;
using namespace TransferConstants;

TransferFirstBall::TransferFirstBall(TransferSubsystem* transfer)
 : m_transfer(transfer)
{
  AddRequirements({transfer});
}

void TransferFirstBall::Initialize()
{
}

void TransferFirstBall::Execute()
{
    m_transfer->SetFeeder(kFeederSpeedIntaking);
    m_transfer->SetTransfer(kTransferSpeedIntaking);
}

bool TransferFirstBall::IsFinished()
{
    return m_transfer->GetFeederPhotoeye();
}

void TransferFirstBall::End(bool interrupted)
{
    m_transfer->SetFeeder(0);
}