#include "commands/TransferLaunch.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TransferConstants;

TransferLaunch::TransferLaunch(TransferSubsystem* subsystem, 
                            bool* turretready, bool* firing, bool* finished,
                            double launchtime)
 : m_transfer(subsystem)
 , m_turretready(turretready)
 , m_firing(firing)
 , m_finished(finished)
 , m_launchtime(launchtime)
{
  AddRequirements({subsystem});
  *m_turretready = false;
  *m_firing = false;
  *m_finished = false;
}

void TransferLaunch::Initialize()
{
    m_timer.Reset();
    m_timer.Stop();
    *m_turretready = false;
    *m_firing = false;
    *m_finished = false;
}

void TransferLaunch::Execute()
{    
    if (*m_turretready)
    {
        *m_firing = true;
        m_timer.Start();
    }

    if (*m_firing)
    {
        m_transfer->SetTransfer(kTransferSpeedFiring);
        m_transfer->SetFeeder(kFeederSpeed);
    }
    else
    {
        m_transfer->SetTransfer(0);
        m_transfer->SetFeeder(0);
    }


    // SmartDashboard::PutBoolean("TEST_READY_TO_FIRE", *m_turretready);
    // SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}

bool TransferLaunch::IsFinished() {
    return m_timer.Get().to<double>() > m_launchtime;
}

void TransferLaunch::End(bool interrupted) {
    *m_finished = true;
    *m_firing = false;
    m_timer.Stop();
    m_transfer->SetFeeder(0);
    m_transfer->SetTransfer(0);

    // SmartDashboard::PutBoolean("TEST_FIRING", *m_firing);
}