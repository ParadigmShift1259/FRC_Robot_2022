#include "subsystems/TransferSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TransferConstants;

TransferSubsystem::TransferSubsystem()
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

void TransferSubsystem::Periodic()
{
    frc::SmartDashboard::PutBoolean("DI_FeederPhotoeye", m_feederphotoeye.Get());
    frc::SmartDashboard::PutBoolean("DI_TransferPhotoeye",  m_transferphotoeye.Get());
}

void TransferSubsystem::SetFeeder(double speed)
{
    m_feedermotor.Set(ControlMode::PercentOutput, speed);
}

void TransferSubsystem::SetTransfer(double speed)
{
    m_transfermotor.Set(ControlMode::PercentOutput, speed);
}

bool TransferSubsystem::AtTransferPosition()
{
    return m_feederphotoeye.Get();
}

bool TransferSubsystem::AtFeederPosition()
{
    return m_transferphotoeye.Get();
}

bool TransferSubsystem::GetTransferPhotoeye() {
    return m_transferphotoeye.Get();
}

bool TransferSubsystem::GetFeederPhotoeye() {
    return m_feederphotoeye.Get();
}
