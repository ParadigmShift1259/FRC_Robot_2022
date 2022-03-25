#include "commands/FireOneBall.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>

using namespace TransferConstants;

//#define SAVE
#ifdef SAVE
FireOneBall::FireOneBall(TransferSubsystem* subsystem)
 : m_transfer(subsystem)
{
  AddRequirements({m_transfer});
}
 #else
 FireOneBall::FireOneBall(ISubsysAccess& subsystemAccess)
  : m_subsystemAccess(subsystemAccess)
{
  AddRequirements({&m_subsystemAccess.GetTransfer());
}
#endif

void FireOneBall::Initialize()
{ 
    m_timer.Reset();
    m_timer.Start();

#ifndef SAVE
    auto& vision = m_subsystemAccess.GetVision();
    vision.CamCapture();
    frc::DataLogManager::Log(fmt::format("Fireing: dist={}, hub angle={}, turret angle={}, RPM={}, hood pos.={}",
                            vision.GetHubDistance(false) * 39.37, 
                            vision.GetHubAngle() * 180/3.14, 
                            m_subsystemAccess.GetTurret().GetCurrentAngle(),
                            m_subsystemAccess.GetFlywheelRpm(),
                            m_subsystemAccess.GetHood().GetServoPosition()
                            ));
#endif
}

void FireOneBall::Execute()
{    
#ifdef SAVE
    m_transfer->SetFeeder(kFeederSpeedFiring);
#else
    m_subsystemAccess.GetTransfer().SetFeeder(kFeederSpeedFiring);
#endif
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
 #ifdef SAVE
   m_transfer->SetFeeder(0.0);
#else
    m_subsystemAccess.GetTransfer().SetFeeder(0.0);
#endif
    m_timer.Stop();
}