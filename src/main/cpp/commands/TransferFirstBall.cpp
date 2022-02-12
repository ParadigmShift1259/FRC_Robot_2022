#include "commands/TransferFirstBall.h"
#include "commands/IntakeTransfer.h"
#include "Constants.h"

using namespace TransferConstants;

TransferFirstBall::TransferFirstBall(TransferSubsystem* transfer, double speed)
 : m_transfer(transfer)
 , m_timer()
 , m_speed(speed)
 {
  AddRequirements({transfer});
}

void TransferFirstBall::Initialize()
{
    m_timer.Start();
}

void TransferFirstBall::Execute()
{
    m_transfer->SetTransfer(kTransferSpeedIntaking);
    m_transfer->SetFeeder(kFeederSpeed);
}

bool TransferFirstBall::IsFinished() {
    return m_transfer->GetFeederPhotoeye();
}

void TransferFirstBall::End(bool interrupted)
{
    m_transfer->SetFeeder(0);
}