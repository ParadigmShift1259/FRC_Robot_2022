
#include "subsystems/ClimberSubsystem.h"

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem() : m_motor(ClimberConstants::kMotorCanId)
{
    m_motor.ConfigFactoryDefault();

    m_motor.SetNeutralMode(NeutralMode::Brake);
    m_motor.SetSensorPhase(true);
    m_motor.SetInverted(false);
    //m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10.0, 1.0);
    //m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10.0, 1.0);

    // m_motor.ConfigNeutralDeadband(0);
    // m_motor.ConfigNominalOutputForward(kNominal);
    // m_motor.ConfigNominalOutputReverse(kNominal * -1.0);
    // m_motor.ConfigPeakOutputForward(kMaxOut);
    // m_motor.ConfigPeakOutputReverse(kMaxOut * -1.0);
}

void ClimberSubsystem::Periodic()
{
}

void ClimberSubsystem::Run(double speed) 
{
    m_motor.Set(ControlMode::PercentOutput, speed * ClimberConstants::kMotorReverseConstant);
}