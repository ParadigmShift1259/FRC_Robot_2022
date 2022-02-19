#include "commands/TransferSecondBall.h"
#include "Constants.h"

using namespace IntakeConstants;
using namespace TransferConstants;

TransferSecondBall::TransferSecondBall(TransferSubsystem& transfer, IntakeSubsystem& intake, double speed)
 : m_transfer(transfer)
 , m_intake(intake)
 , m_speed(speed)
 , m_photoeyeCount(0)
 {
  AddRequirements({&transfer, &intake});
}

void TransferSecondBall::Initialize()
{
    m_photoeyeCount = 0;
}

void TransferSecondBall::Execute()
{
    m_transfer.SetTransfer(kTransferSpeedIntaking);
    m_intake.Set(kIngestHigh);
}

bool TransferSecondBall::IsFinished() {
    if (m_transfer.GetTransferPhotoeye()) {
        m_photoeyeCount++;
    }

    return m_transfer.GetTransferPhotoeye();
}

void TransferSecondBall::End(bool interrupted)
{
    m_transfer.SetTransfer(0);
    m_intake.Set(0);
}
