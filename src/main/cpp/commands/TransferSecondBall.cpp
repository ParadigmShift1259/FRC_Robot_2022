#include "commands/TransferSecondBall.h"
#include "Constants.h"

using namespace TransferConstants;

TransferSecondBall::TransferSecondBall(TransportSubsystem* subsystem)
 : m_transport(subsystem)
 , m_photoeyeCount(0)
 {
  AddRequirements({subsystem});
}

void TransferSecondBall::Initialize()
{
    m_photoeyeCount = 0;
}

void TransferSecondBall::Execute()
{

}

bool TransferSecondBall::IsFinished() {
    if (m_transport->GetTransferPhotoeye()) {
        m_photoeyeCount ++;
    }

    return m_photoeyeCount > 1 && m_transport->GetTransferPhotoeye();
}

void TransferSecondBall::End(bool interrupted)
{
    m_transport->SetTransfer(0);
}