/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace std;
using namespace frc;

DriveSubsystem::DriveSubsystem(Team1259::Gyro *gyro)
    : m_frontLeft
      {
          kFrontLeftDriveMotorPort
        , kFrontLeftTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kFrontLeftPWM
        , kFrontLeftDriveMotorReversed
        , kFrontLeftOffset
        , string("FrontLeft")
      }
    , m_frontRight
      {
          kFrontRightDriveMotorPort
        , kFrontRightTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kFrontRightPWM
        , kFrontRightDriveMotorReversed
        , kFrontRightOffset
        , string("FrontRight")
      }
    , m_rearRight
      {
          kRearRightDriveMotorPort
        , kRearRightTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kRearRightPWM
        , kRearRightDriveMotorReversed
        , kRearRightOffset
        , string("RearRight")
      }
    , m_rearLeft
      {
          kRearLeftDriveMotorPort
        , kRearLeftTurningMotorPort
        , [this](CANifier::PWMChannel channel){ return PWMToPulseWidth(channel); } 
        , kRearLeftPWM
        , kRearLeftDriveMotorReversed
        , kRearLeftOffset
        , string("RearLeft")
      }
    , m_canifier(kCanifierID)
    , m_gyro(gyro)
    , m_odometry{kDriveKinematics, m_gyro->GetHeadingAsRot2d(), Pose2d()}
{

    #ifdef MANUAL_MODULE_STATES
    SmartDashboard::PutNumber("T_D_MFL", 0);
    SmartDashboard::PutNumber("T_D_MFR", 0);
    SmartDashboard::PutNumber("T_D_MRR", 0);
    SmartDashboard::PutNumber("T_D_MRL", 0);
    SmartDashboard::PutNumber("T_D_MFLV", 0);
    SmartDashboard::PutNumber("T_D_MFRV", 0);
    SmartDashboard::PutNumber("T_D_MRRV", 0);
    SmartDashboard::PutNumber("T_D_MRLV", 0);
    #endif

    #ifdef TUNE_ROTATION_DRIVE
    SmartDashboard::PutNumber("T_D_RP", 0);
    SmartDashboard::PutNumber("T_D_RI", 0);
    SmartDashboard::PutNumber("T_D_RD", 0);
    SmartDashboard::PutNumber("T_D_RMax", 0);
    SmartDashboard::PutNumber("T_D_RTMax", 0);
    #endif

    m_rotationPIDController.SetTolerance(kRotationDriveTolerance);
    m_rotationPIDController.SetIntegratorRange(0, kRotationDriveIMaxRange);

    m_lastHeading = 0;
    m_rotationalInput = true;
    m_StateHist.reserve(10000);
    m_StateHist.clear(); // clear() does not delatocate memory
}

void DriveSubsystem::Periodic()
{
    m_frontLeft.Periodic();
    m_frontRight.Periodic();
    m_rearRight.Periodic();
    m_rearLeft.Periodic();
    m_odometry.Update(m_gyro->GetHeadingAsRot2d()
                , m_frontLeft.GetState()
                , m_frontRight.GetState()
                , m_rearLeft.GetState()
                , m_rearRight.GetState());

    frc::Pose2d pose = m_odometry.GetPose();
    frc::Trajectory::State state;
    state.t = m_timer.GetFPGATimestamp();
    state.pose = pose;
	auto& prevState = m_StateHist.back();
    state.velocity = (pose - prevState.pose).Translation().Norm() / (state.t - prevState.t);
    state.acceleration = (state.velocity - prevState.velocity) / (state.t - prevState.t);
    m_velocity = (double)state.velocity;
    m_acceleration = (double)state.acceleration;

    m_StateHist.push_back(state);
}

void DriveSubsystem::RotationDrive(meters_per_second_t xSpeed
                                , meters_per_second_t ySpeed
                                , radian_t rot
                                , bool fieldRelative) 
{  
    double error = rot.to<double>() - m_gyro->GetHeadingAsRot2d().Radians().to<double>();
    double desiredSet = Util::NegPiToPiRads(error);
    double max = kRotationDriveMaxSpeed;
    double maxTurn = kRotationDriveDirectionLimit;

    #ifdef TUNE_ROTATION_DRIVE
    double P = SmartDashboard::GetNumber("T_D_RP", 0);
    double I = SmartDashboard::GetNumber("T_D_RI", 0);
    double D = SmartDashboard::GetNumber("T_D_RD", 0);
    double m = SmartDashboard::GetNumber("T_D_RMax", 0);
    double mTurn = SmartDashboard::GetNumber("T_D_RTMax", 0);

    m_rotationPIDController.SetP(P);
    m_rotationPIDController.SetI(I);
    m_rotationPIDController.SetD(D);
    max = m;
    maxTurn = mTurn;
    #endif

    double desiredTurnRate = m_rotationPIDController.Calculate(0, desiredSet);

    double currentTurnRate = m_gyro->GetTurnRate() * wpi::numbers::pi / 180;

    // Prevent sharp turning if already fast going in the opposite direction
    if ((abs(currentTurnRate) >= maxTurn) && (signbit(desiredTurnRate) != signbit(currentTurnRate)))
        desiredTurnRate *= -1.0;

    // Power limiting
    if (abs(desiredTurnRate) > max)
        desiredTurnRate = signbit(desiredTurnRate) ? max * -1.0 : max;

    Drive(xSpeed, ySpeed, radians_per_second_t(desiredTurnRate), fieldRelative);
}

void DriveSubsystem::RotationDrive(meters_per_second_t xSpeed
                                , meters_per_second_t ySpeed
                                , double xRot
                                , double yRot
                                , bool fieldRelative) 
{
    if (xRot != 0 || yRot != 0)
    {
        m_rotationalInput = true;
        RotationDrive(xSpeed, ySpeed, radian_t(atan2f(yRot, xRot)), fieldRelative);
    }
    else
        Drive(xSpeed, ySpeed, radians_per_second_t(0), fieldRelative);
    
}

void DriveSubsystem::HeadingDrive(meters_per_second_t xSpeed
                                , meters_per_second_t ySpeed
                                , radians_per_second_t rot
                                , bool fieldRelative)
{
    if (xSpeed.to<double>() == 0 && ySpeed.to<double>() == 0 &&   rot.to<double>() == 0)
    {
        Drive(xSpeed, ySpeed, rot, fieldRelative);
        return;
    }

    if (rot.to<double>() == 0 && m_rotationalInput)
    {
        m_rotationalInput = false;
        UpdateLastHeading();
    }
    else if (rot.to<double>() != 0)
        m_rotationalInput = true;
    
    if (!m_rotationalInput && (xSpeed.to<double>() != 0 || ySpeed.to<double>() != 0))
        RotationDrive(xSpeed, ySpeed, radian_t(m_lastHeading), fieldRelative);
    else
        Drive(xSpeed, ySpeed, rot, fieldRelative);
}

void DriveSubsystem::Drive(meters_per_second_t xSpeed
                        , meters_per_second_t ySpeed
                        , radians_per_second_t rot
                        , bool fieldRelative)
{
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative)
        chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro->GetHeadingAsRot2d());
    else
        chassisSpeeds = ChassisSpeeds{xSpeed, ySpeed, rot};

    auto states = kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);

    kDriveKinematics.DesaturateWheelSpeeds(&states, kDriveSpeed);
    
    #ifdef MANUAL_MODULE_STATES
    states[kFrontLeft].angle = Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MFL", 0.0)));
    states[kFrontRight].angle = Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MFR", 0.0)));
    states[kRearRight].angle = Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MRR", 0.0)));
    states[kRearLeft].angle = Rotation2d(radian_t(SmartDashboard::GetNumber("T_D_MRL", 0.0)));
    states[kFrontLeft].speed = SmartDashboard::GetNumber("T_D_MFLV", 0.0) * 1_mps;
    states[kFrontRight].speed = SmartDashboard::GetNumber("T_D_MFRV", 0.0) * 1_mps;
    states[kRearRight].speed = SmartDashboard::GetNumber("T_D_MRRV", 0.0) * 1_mps;
    states[kRearLeft].speed = SmartDashboard::GetNumber("T_D_MRLV", 0.0) * 1_mps;
    #endif

    m_frontLeft.SetDesiredState(states[kFrontLeft]);
    m_frontRight.SetDesiredState(states[kFrontRight]);
    m_rearLeft.SetDesiredState(states[kRearLeft]);
    m_rearRight.SetDesiredState(states[kRearRight]);
}

void DriveSubsystem::SetModuleStates(SwerveModuleStates desiredStates)
{
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, AutoConstants::kMaxSpeed);
    m_frontLeft.SetDesiredState(desiredStates[kFrontLeft]);
    m_frontRight.SetDesiredState(desiredStates[kFrontRight]);
    m_rearRight.SetDesiredState(desiredStates[kRearRight]);
    m_rearLeft.SetDesiredState(desiredStates[kRearLeft]);
}

void DriveSubsystem::UpdateLastHeading()
{
    m_lastHeading = m_gyro->GetHeadingAsRot2d().Radians().to<double>();
}

void DriveSubsystem::ResetEncoders()
{
    m_frontLeft.ResetEncoders();
    m_frontRight.ResetEncoders();
    m_rearRight.ResetEncoders();
    m_rearLeft.ResetEncoders();
}

Pose2d DriveSubsystem::GetPose()
{
    return m_odometry.GetPose();
}

double DriveSubsystem::PWMToPulseWidth(CANifier::PWMChannel pwmChannel)
{
    double dutyCycleAndPeriod[2];
    m_canifier.GetPWMInput(pwmChannel, dutyCycleAndPeriod);
    return dutyCycleAndPeriod[0] * dutyCycleAndPeriod[1] / kPulseWidthToZeroOne;
}

void DriveSubsystem::ResetOdometry(Pose2d pose)
{
    m_odometry.ResetPosition(pose, m_gyro->GetHeadingAsRot2d());
}

void DriveSubsystem::ResetRelativeToAbsolute()
{
    m_frontLeft.ResetRelativeToAbsolute();
    m_frontRight.ResetRelativeToAbsolute();
    m_rearRight.ResetRelativeToAbsolute();
    m_rearLeft.ResetRelativeToAbsolute();
}

void DriveSubsystem::WheelsForward()
{
    static SwerveModuleState zeroState { 0_mps, 0_deg };
    // printf("DriveSubsystem::WheelsForward() called");
    m_frontLeft.SetDesiredState(zeroState);
    m_frontRight.SetDesiredState(zeroState);
    m_rearRight.SetDesiredState(zeroState);
    m_rearLeft.SetDesiredState(zeroState);
}
