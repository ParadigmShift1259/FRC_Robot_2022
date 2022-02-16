#include "commands/TransferSecondBall.h"
#include "Constants.h"

using namespace TransferConstants;

TransferSecondBall::TransferSecondBall(TransferSubsystem* subsystem, double speed)
 : m_transfer(subsystem)
 , m_timer()
 , m_speed(speed)
 , m_photoeyeCount(0)
 {
  AddRequirements({subsystem});
}

void TransferSecondBall::Initialize()
{
    m_timer.Start();
    m_photoeyeCount = 0;
}

void TransferSecondBall::Execute()
{

}

bool TransferSecondBall::IsFinished() {
    if (m_transfer->GetTransferPhotoeye()) {
        m_photoeyeCount++;
    }

    return m_transfer->GetTransferPhotoeye();
}

void TransferSecondBall::End(bool interrupted)
{
    m_transfer->SetTransfer(0);
}