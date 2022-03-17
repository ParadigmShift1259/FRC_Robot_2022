#include "commands/TransferFire.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TransferConstants;

TransferFire::TransferFire(TransferSubsystem* subsystem, 
                            bool* turretready, bool* firing, bool* finished,
                            double launchtime)
 : m_transfer(subsystem)
 , m_turretready(turretready)
 , m_firing(firing)
 , m_finished(finished)
 , m_launchtime(launchtime)
{
  AddRequirements({subsystem});
  *m_firing = false;
  *m_finished = false;
}

void TransferFire::Initialize()
{
#define COMMAND_TIMING
#ifdef COMMAND_TIMING
   m_startTime = m_timer.GetFPGATimestamp().to<double>();
   printf("timestamp start transfer fire %.3f\n", m_startTime);
#endif
 
    m_timer.Reset();
    m_timer.Stop();
    *m_firing = false;
    *m_finished = false;
}

void TransferFire::Execute()
{    
    if (*m_turretready)
    {
        *m_firing = true;
        m_timer.Start();
    }

    if (*m_firing)
    {
        //m_transfer->SetFeeder(kFeederSpeedFiring);
        m_transfer->SetFeeder(kFeederSpeedFiring);
        if (m_timer.Get().to<double>() >= kTimePassed)
        {
            m_transfer->SetTransfer(kTransferSpeedFiring);
        }        
    }
    else
    {
        m_transfer->SetTransfer(0.0);
        m_transfer->SetFeeder(0.0);
    }

    frc::SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
    // SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}

bool TransferFire::IsFinished()
{
    return m_timer.Get().to<double>() > m_launchtime;
}

void TransferFire::End(bool interrupted)
{
    *m_finished = true;
    frc::SmartDashboard::PutBoolean("TEST_FIRE_FINISIHED", *m_finished);
    *m_firing = false;
    m_timer.Stop();
    m_transfer->SetFeeder(0);
    m_transfer->SetTransfer(0);

#ifdef COMMAND_TIMING
    auto endTime = m_timer.GetFPGATimestamp().to<double>();
    auto elapsed = endTime - m_startTime;
    printf("timestamp end transfer fire %.3f elapsed %.3f\n", endTime, elapsed);
#endif

    // SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}