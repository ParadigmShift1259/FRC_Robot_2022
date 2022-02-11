#include "commands/TransferCmd.h"
#include "Constants.h"

using namespace TransferConstants;

TransferCmd::TransferCmd(TransferSubsystem* subsystem, double speed)
 : m_transfer(subsystem)
 , m_timer()
 , m_speed(speed)
 {
  AddRequirements({subsystem});
}

void TransferCmd::Initialize()
{
    m_timer.Start();
}

void TransferCmd::Execute()
{
    if (m_timer.Get().to<double>() <= kTimePassed * 1) {
        m_transfer->SetTransfer(m_speed);
    }
    else if (m_timer.Get().to<double>() <= kTimePassed * 2) {
        m_transfer->SetTransfer(0);
    }
    else if (m_timer.Get().to<double>() <= kTimePassed * 3) {
        m_transfer->SetTransfer(m_speed * -1.0);
    }
    else if (m_timer.Get().to<double>() <= kTimePassed * 4) {
        m_transfer->SetTransfer(0);
    }
    else if (m_timer.Get().to<double>() > kTimePassed * 4) {
        m_timer.Reset();
    }
}

void TransferCmd::End(bool interrupted)
{
    m_transfer->SetTransfer(0);
}