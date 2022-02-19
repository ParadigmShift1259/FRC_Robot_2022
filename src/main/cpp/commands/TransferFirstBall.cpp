#include "commands/TransferFirstBall.h"
#include "commands/IntakeTransfer.h"
#include "Constants.h"

using namespace IntakeConstants;
using namespace TransferConstants;

TransferFirstBall::TransferFirstBall(TransferSubsystem& transfer, IntakeSubsystem& intake, double speed)
 : m_transfer(transfer)
 , m_intake(intake)
 , m_speed(speed)
{
  AddRequirements({&transfer, &intake});
}

void TransferFirstBall::Initialize()
{
    m_bRunning = false;
    printf("TransferFirstBall::Initialize m_bRunning %d\n", m_bRunning);
}

void TransferFirstBall::Execute()
{
    m_bRunning = true;
    m_transfer.SetFeeder(kFeederSpeed);
    m_transfer.SetTransfer(kTransferSpeedIntaking);
    m_intake.Set(kIngestHigh);
    printf("TransferFirstBall::Execute m_bRunning %d\n", m_bRunning);
}

bool TransferFirstBall::IsFinished()
{
    bool bPe =  m_transfer.GetFeederPhotoeye();
    printf("TransferFirstBall::IsFinished m_bRunning %d PE %d\n", m_bRunning, bPe);
//    return !m_bRunning || m_transfer.GetFeederPhotoeye();
//    return !m_bRunning || bPe;
    return bPe;
}

void TransferFirstBall::End(bool interrupted)
{
   m_bRunning = false;
   printf("TransferFirstBall::End m_bRunning %d\n", m_bRunning);
   m_transfer.SetFeeder(0);
}