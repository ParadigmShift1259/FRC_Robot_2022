#include "subsystems/TransportSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TransferConstants;

TransportSubsystem::TransportSubsystem()
    : m_feedermotor(kFeederCANid)
    , m_transfermotor(kTransferCANid)
    , m_feederphotoeye(kFeederInputChannel) 
    , m_transferphotoeye(kTransferInputChannel)
{
    m_transfermotor.SetNeutralMode(NeutralMode::Brake);
    m_transfermotor.SetInverted(kTransferInverted);
    m_transfermotor.ConfigOpenloopRamp(kTransferRampRate, kTimeout);

    m_feedermotor.SetNeutralMode(NeutralMode::Brake);
    m_feedermotor.SetInverted(kFeederInverted);
}

void TransportSubsystem::Periodic()
{
    frc::SmartDashboard::PutBoolean("DI_FeederPhotoeye", m_feederphotoeye.Get());
    frc::SmartDashboard::PutBoolean("DI_TransferPhotoeye",  m_transferphotoeye.Get());
}

void TransportSubsystem::SetFeeder(double speed)
{
    m_feedermotor.Set(ControlMode::PercentOutput, speed);
}

void TransportSubsystem::SetTransfer(double speed)
{
    m_transfermotor.Set(ControlMode::PercentOutput, speed);
}

bool TransportSubsystem::AtTransferPosition()
{
    return m_feederphotoeye.Get();
}

bool TransportSubsystem::AtFeederPosition()
{
    return m_transferphotoeye.Get();
}

bool TransportSubsystem::GetTransferPhotoeye() {
    return m_transferphotoeye.Get();
}

bool TransportSubsystem::GetFeederPhotoeye() {
    return m_feederphotoeye.Get();
}
