#include "commands/TransferSecondBall.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
    frc::SmartDashboard::PutNumber("Transfer PhotoeyeCount", m_photoeyeCount);
    m_timer.Start();
    m_photoeyeCount = 0;
    m_bRunning = false;
}

void TransferSecondBall::Execute()
{
    m_bRunning = true;
    m_transfer.SetTransfer(kTransferSpeedIntaking);
    m_intake.Set(kIngestHigh);
    printf("Transfer PhotoeyeCount, %d", m_photoeyeCount);
    frc::SmartDashboard::PutNumber("Transfer PhotoeyeCount", m_photoeyeCount);
    printf("TransferSecondBall::Execute m_bRunning %d\n", m_bRunning);
}

bool TransferSecondBall::IsFinished() {
    if (m_transfer.GetTransferPhotoeye()) {
        m_photoeyeCount++;
    }
    frc::SmartDashboard::PutNumber("Transfer PhotoeyeCount", m_photoeyeCount);

    return !m_bRunning || m_transfer.GetTransferPhotoeye();
}

void TransferSecondBall::End(bool interrupted)
{
    m_bRunning = false;
    printf("TransferSecondBall::End m_bRunning %d\n", m_bRunning);
    m_transfer.SetTransfer(0);
    m_intake.Set(0);
}