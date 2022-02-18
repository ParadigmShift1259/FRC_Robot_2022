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
}

void TransferFirstBall::Execute()
{
    m_transfer.SetFeeder(kFeederSpeed);
    m_transfer.SetTransfer(kTransferSpeedIntaking);
    m_intake.Set(kIngestHigh);
}

bool TransferFirstBall::IsFinished()
{
    return m_transfer.GetFeederPhotoeye();
}

void TransferFirstBall::End(bool interrupted)
{
    m_transfer.SetFeeder(0);
}