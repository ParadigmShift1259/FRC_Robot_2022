#include "commands/TransferSecondBall.h"
#include "Constants.h"

using namespace IntakeConstants;
using namespace TransferConstants;

TransferSecondBall::TransferSecondBall(ISubsysAccess& subsysAccess)
 : m_subSysAccess(subsysAccess)
 , m_transfer(subsysAccess.GetTransfer())
 , m_intake(subsysAccess.GetIntake())
{
  AddRequirements({&m_transfer, &m_intake});
}

void TransferSecondBall::Initialize()
{
}

void TransferSecondBall::Execute()
{
    m_transfer.SetTransfer(kTransferSpeedIntaking);
    m_intake.Set(kIngestHigh);
}

bool TransferSecondBall::IsFinished()
{
    return (m_transfer.GetTransferPhotoeye() || m_subSysAccess.OnlyOneBall());
}

void TransferSecondBall::End(bool interrupted)
{
    m_transfer.SetTransfer(0);
    m_intake.Set(0);
}
