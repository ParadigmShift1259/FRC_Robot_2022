/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorChannel, 
                           int turningMotorChannel,
                           const int turningEncoderPort,
                           bool driveMotorReversed,
                           double offset,
                           const std::string& name)
    : m_offset(offset)
    , m_name(name)
    , m_driveMotor(driveMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_drivePIDLoader("SM", kDriveAdjust, kDriveP, kDriveI, kDriveD, 0, 0, kDriveFF, 1.0, 0.0)
    , m_turnPIDLoader("SMT", kTurnAdjust, kTurnP, kTurnI, kTurnD, kTurnIZ, kTurnIA)
    , m_turningEncoder(turningEncoderPort)
{
    m_driveMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);
    m_turningMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);

    m_driveMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);

    // Set up GetVelocity() to return meters per sec instead of RPM
    m_driveEncoder.SetVelocityConversionFactor(wpi::numbers::pi * ModuleConstants::kWheelDiameterMeters / (ModuleConstants::kDriveGearRatio * 60.0));
    m_turnRelativeEncoder.SetPositionConversionFactor(2 * wpi::numbers::pi / ModuleConstants::kTurnMotorRevsPerWheelRev);
    
    m_driveMotor.SetInverted(driveMotorReversed);
    m_turningMotor.SetInverted(false);

    m_drivePIDLoader.Load(m_drivePIDController);
    m_turnPIDLoader.Load(m_turnPIDController);

    double initPosition = CalcAbsoluteAngle();
    m_turnRelativeEncoder.SetPosition(initPosition); // Tell the encoder where the absolute encoder is
}

SwerveModuleState SwerveModule::GetState()
{
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, Rotation2d(radian_t(CalcAbsoluteAngle()))};
    // double angle = m_turnRelativeEncoder.GetPosition();
    // return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::Periodic()
{
    double absAngle = CalcAbsoluteAngle();
    SmartDashboard::PutNumber("D_SM_Rel " + m_name, m_turnRelativeEncoder.GetPosition());
    SmartDashboard::PutNumber("D_SM_Abs " + m_name, absAngle);
    SmartDashboard::PutNumber("D_SM_AbsDiff " + m_name, m_turnRelativeEncoder.GetPosition() - absAngle);
}

void SwerveModule::SetDesiredState(SwerveModuleState &state)
{
    // Retrieving PID values from SmartDashboard if enabled
    // m_drivePIDLoader.LoadFromNetworkTable(m_drivePIDController);
    // m_turnPIDLoader.LoadFromNetworkTable(m_turnPIDController);

    // Find NEO relative encoder position
    double currentPosition = m_turnRelativeEncoder.GetPosition();

    // Calculate new turn position given current Neo position, current absolute encoder position, and desired state position
    bool bOutputReverse = false;
    double minTurnRads = MinTurnRads(currentPosition, state.angle.Radians().to<double>(), bOutputReverse);
    double direction = 1.0;
    // Inverts the drive motor if going in the "backwards" direction on the swerve module
    if (bOutputReverse)
        direction = -1.0;

    // Set position reference of turnPIDController
    double newPosition = currentPosition + minTurnRads;
    if (state.speed != 0_mps) {
        #ifdef DISABLE_DRIVE
        m_drivePIDController.SetReference(0, ControlType::kVelocity);
        #else
        m_drivePIDController.SetReference(direction * state.speed.to<double>(), CANSparkMax::ControlType::kVelocity);
        #endif
    }
    else
        m_drivePIDController.SetReference(0, CANSparkMax::ControlType::kVoltage);

    // Set the angle unless module is coming to a full stop
    if (state.speed != 0_mps)
    {
        m_turnPIDController.SetReference(newPosition, CANSparkMax::ControlType::kPosition);
        //printf("Position: %f\n", newPosition);
    }

}

void SwerveModule::ResetEncoders()
{
    m_driveEncoder.SetPosition(0.0); 
}

double SwerveModule::CalcAbsoluteAngle()
{
    double angle = fmod(m_turningEncoder.GetVoltage() * DriveConstants::kTurnCalcAbsoluteAngle - m_offset + 2 * wpi::numbers::pi, 2 * wpi::numbers::pi);
    angle = 2 * wpi::numbers::pi - angle;
    return angle;
}

void SwerveModule::ResetRelativeToAbsolute()
{
    m_turnRelativeEncoder.SetPosition(CalcAbsoluteAngle());
}

double SwerveModule::MinTurnRads(double init, double final, bool& bOutputReverse)
{
    init = Util::ZeroTo2PiRads(init);
    final = Util::ZeroTo2PiRads(final);

    // The shortest turn angle may be acheived by reversing the motor output direction
    double angle1 = final - init;
    double angle2 = final + wpi::numbers::pi - init;

    angle1 = Util::NegPiToPiRads(angle1);
    angle2 = Util::NegPiToPiRads(angle2);

    return angle1;
}
