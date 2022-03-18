/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"


SwerveModule::SwerveModule(int driveMotorChannel, 
                           int turningMotorChannel,
                           GetPulseWidthCallback pulseWidthCallback,
                           CANifier::PWMChannel pwmChannel,
                           bool driveMotorReversed,
                           double offset,
                           const std::string& name)
    : m_offset(offset)
    , m_name(name)
    , m_driveMotor(driveMotorChannel)
    , m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_drivePIDLoader("SM", kDriveAdjust, kDriveP, kDriveI, kDriveD, kDriveFF)
    , m_turnPIDLoader("SM", kTurnAdjust, kTurnP, kTurnI, kTurnD, kTurnIZ, kTurnIA)
    , m_pulseWidthCallback(pulseWidthCallback)
    , m_pwmChannel(pwmChannel)
{
    StatorCurrentLimitConfiguration statorLimit { true, kMotorCurrentLimit, kMotorCurrentLimit, 2 };
    m_driveMotor.ConfigStatorCurrentLimit(statorLimit);
    SupplyCurrentLimitConfiguration supplyLimit { true, kMotorCurrentLimit, kMotorCurrentLimit, 2 };
    m_driveMotor.ConfigSupplyCurrentLimit(supplyLimit);
    m_turningMotor.SetSmartCurrentLimit(kMotorCurrentLimit);

    // Set up GetVelocity() to return meters per sec instead of RPM
    m_turnRelativeEncoder.SetPositionConversionFactor(2.0 * wpi::numbers::pi / kTurnMotorRevsPerWheelRev);
    
    m_driveMotor.SetInverted(driveMotorReversed ? TalonFXInvertType::CounterClockwise : TalonFXInvertType::Clockwise);
    m_turningMotor.SetInverted(false);
    m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    m_turnPIDController.SetFeedbackDevice(m_turnRelativeEncoder);
    
    m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_driveMotor.SetNeutralMode(NeutralMode::Brake);

    m_drivePIDLoader.Load(m_driveMotor);
    m_turnPIDLoader.Load(m_turnPIDController);

    m_timer.Reset();
    m_timer.Start();
}

void SwerveModule::Periodic()
{
    if (m_timer.Get() < 5_s)
        ResetRelativeToAbsolute();

    //double absAngle = 
    CalcAbsoluteAngle();
    //SmartDashboard::PutNumber("D_SM_Rel " + m_name, m_turnRelativeEncoder.GetPosition());
    //SmartDashboard::PutNumber("D_SM_Abs " + m_name, absAngle);
    //SmartDashboard::PutNumber("D_SM_AbsDiff " + m_name, m_turnRelativeEncoder.GetPosition() - absAngle);
    // SmartDashboard::PutNumber("D_SM_MPS " + m_name, CalcMetersPerSec().to<double>());
    // SmartDashboard::PutNumber("D_SM_IError " + m_name, m_turnPIDController.GetIAccum());
    // SmartDashboard::PutNumber("D_SM_TP100MS " + m_name, m_driveMotor.GetSelectedSensorVelocity());
}

SwerveModuleState SwerveModule::GetState()
{
    /// Why do we find the absolute angle here instead of the relative angle?
//    return { CalcMetersPerSec(), Rotation2d(radian_t(CalcAbsoluteAngle()))};
    return { CalcMetersPerSec(), Rotation2d(radian_t(m_turnRelativeEncoder.GetPosition()))};
}

void SwerveModule::SetDesiredState(SwerveModuleState &state)
{
    // Retrieving PID values from SmartDashboard if enabled
    m_drivePIDLoader.LoadFromNetworkTable(m_driveMotor);
    m_turnPIDLoader.LoadFromNetworkTable(m_turnPIDController);

    // Calculate optimized state based on current relative angle
    Rotation2d currentAngle = Rotation2d{ m_turnRelativeEncoder.GetPosition() * 1_rad};
    frc::SwerveModuleState optimizedState = SwerveModuleState::Optimize(state, currentAngle);

    if (optimizedState.speed != 0_mps) {
        #ifdef DISABLE_DRIVE
        m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
        #else
        m_driveMotor.Set(TalonFXControlMode::Velocity, CalcTicksPer100Ms(optimizedState.speed));
        #endif
    }
    else
        m_driveMotor.Set(TalonFXControlMode::PercentOutput, 0.0);

    // Set the angle unless module is coming to a full stop
    if (optimizedState.speed.to<double>() != 0.0)
        m_turnPIDController.SetReference(optimizedState.angle.Radians().to<double>(), CANSparkMax::ControlType::kPosition);

    // SmartDashboard::PutNumber("D_SM_SetpointMPS " + m_name, state.speed.to<double>());
    // SmartDashboard::PutNumber("D_SM_Error " + m_name, newPosition - m_turnRelativeEncoder.GetPosition());
}

void SwerveModule::ResetEncoders()
{
    m_driveMotor.SetSelectedSensorPosition(0.0);
}

double SwerveModule::CalcAbsoluteAngle()
{
    double pulseWidth = m_pulseWidthCallback(m_pwmChannel);
    // Pulse Width per rotation is not equal for all encoders. Some are 0 - 3865, some are 0 - 4096
    SmartDashboard::PutNumber("D_SM_PW " + m_name, pulseWidth);
    return fmod((pulseWidth - m_offset) * DriveConstants::kPulseWidthToRadians + 2.0 * wpi::numbers::pi, 2.0 * wpi::numbers::pi);
    // SmartDashboard::PutNumber("D_SM_AA " + m_name, absAngle);
    // Convert CW to CCW? absAngle = 2.0 * wpi::numbers::pi - absAngle;
}

void SwerveModule::ResetRelativeToAbsolute()
{
    m_turnRelativeEncoder.SetPosition(CalcAbsoluteAngle());
}

meters_per_second_t SwerveModule::CalcMetersPerSec()
{
   double ticksPer100ms = m_driveMotor.GetSelectedSensorVelocity();
   return meters_per_second_t(kDriveEncoderMetersPerSec * ticksPer100ms);
}

double SwerveModule::CalcTicksPer100Ms(meters_per_second_t speed)
{
   return speed.to<double>() / kDriveEncoderMetersPerSec;
}