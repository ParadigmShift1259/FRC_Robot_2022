#include "commands/FireOneBall.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TransferConstants;

FireOneBall::FireOneBall(TransferSubsystem* subsystem)
 : m_transfer(subsystem)
{
  AddRequirements({subsystem});
}

void FireOneBall::Initialize()
{ 
    m_timer.Reset();
    m_timer.Start();
}

void FireOneBall::Execute()
{    
    m_transfer->SetFeeder(kFeederSpeedFiring);
}

bool FireOneBall::IsFinished()
{
    auto delay = frc::SmartDashboard::GetNumber("FireOnedelay", 0.300);
    // Give time for the second ball transfer to the feeder
    //const auto delay = 0.400;
    return m_timer.Get() > second_t(delay);
}

void FireOneBall::End(bool interrupted)
{
    m_transfer->SetFeeder(0.0);
    // m_transfer->SetTransfer(0.0);
    m_timer.Stop();
}