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
    m_timer.Start();
}

void FireOneBall::Execute()
{    
    m_transfer->SetFeeder(kFeederSpeedIntaking);
}

bool FireOneBall::IsFinished()
{
    return m_timer.Get() > 0.100_s;
}

void FireOneBall::End(bool interrupted)
{
    m_transfer->SetFeeder(0.0);
    m_timer.Stop();
}