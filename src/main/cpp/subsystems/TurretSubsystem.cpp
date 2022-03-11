
#include "subsystems/TurretSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace TurretConstants;

//#define TUNE_TURRET_PID

TurretSubsystem::TurretSubsystem(Team1259::Gyro *gyro) 
    : m_turretmotor(kMotorPort)
    , m_gyro(gyro)
    , m_absEnc(0)
{
    m_turretmotor.ConfigFactoryDefault();

    m_turretmotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeout);
    m_turretmotor.SetNeutralMode(NeutralMode::Brake);
    m_turretmotor.SetSensorPhase(kSensorPhase);
    m_turretmotor.SetInverted(kInverted);
    m_turretmotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10.0, 1.0);
    //m_turretmotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10.0, 1.0);

    m_turretmotor.Config_kP(0, kP, kTimeout);
    m_turretmotor.Config_kI(0, kI, kTimeout);
    m_turretmotor.Config_kD(0, kD, kTimeout);
    m_turretmotor.Config_kF(0, kF, kTimeout);
    m_turretmotor.Config_IntegralZone(0, 1000.0);
    m_turretmotor.ConfigMaxIntegralAccumulator(0, 50000.0);
    m_turretmotor.SetIntegralAccumulator(0.0, 0);
    m_turretmotor.ConfigMotionSCurveStrength(1.0);

    m_turretmotor.ConfigNeutralDeadband(kNeutralDeadband, kTimeout);
    m_turretmotor.ConfigNominalOutputForward(kNominal, kTimeout);
    m_turretmotor.ConfigNominalOutputReverse(kNominal * -1.0, kTimeout);
    m_turretmotor.ConfigPeakOutputForward(kMaxOut, kTimeout);
    m_turretmotor.ConfigPeakOutputReverse(kMaxOut * -1.0, kTimeout);
    //m_turretmotor.ConfigClosedloopRamp()
    //m_turretmotor.ConfigAllowableClosedloopError(0, DegreesToTicks(kDegreePIDStopRange), kTimeout);
    m_turretmotor.ConfigAllowableClosedloopError(0, 30.0, kTimeout);    // Approx 90 ticks per degree **** WHAT DOES THIS DO WHEN USING MOTION MAGIC? ****
    m_turretmotor.ConfigMotionCruiseVelocity(DegreesToTicks(kMMCruiseVel/10), kTimeout);  // encoder ticks per 100ms 
    m_turretmotor.ConfigMotionAcceleration(DegreesToTicks(kMMAccel/10), kTimeout);     // encoder ticks per 100ms per sec

    m_turretmotor.SetSelectedSensorPosition(DegreesToTicks(kStartingPositionDegrees), 0, kTimeout);
    m_turretmotor.ConfigForwardSoftLimitThreshold(5595);
    m_turretmotor.ConfigReverseSoftLimitThreshold(-5595);
    m_turretmotor.ConfigForwardSoftLimitEnable(true);
    m_turretmotor.ConfigReverseSoftLimitEnable(true);

#define USE_MOTION_MAGIC
#ifdef USE_MOTION_MAGIC
    m_turretmotor.Set(ControlMode::MotionMagic, DegreesToTicks(kStartingPositionDegrees));
#else
    m_turretmotor.Set(ControlMode::Position, DegreesToTicks(kStartingPositionDegrees));
#endif
    //m_turretmotor.Set(ControlMode::PercentOutput, 0.1);
    m_currentAngle = kStartingPositionDegrees;
    m_startingPos = m_absEnc.GetValue();
    frc::SmartDashboard::PutNumber("TurretStartingPos", m_startingPos);

#ifdef TUNE_TURRET_PID
    frc::SmartDashboard::PutNumber("TurretP", kP);
    frc::SmartDashboard::PutNumber("TurretI", kI);
    frc::SmartDashboard::PutNumber("TurretD", kD);
    frc::SmartDashboard::PutNumber("TurretF", kF);

    frc::SmartDashboard::PutNumber("TurretIzone", 1000.0);
    frc::SmartDashboard::PutNumber("TurretCruiseV", kMMCruiseVel);
    frc::SmartDashboard::PutNumber("TurretAccel", kMMAccel);
    frc::SmartDashboard::PutNumber("TurretScurve", 1.0);
#endif

    //frc::SmartDashboard::PutNumber("TurretDeadbandPercent", kNeutralDeadband);
}

constexpr double kDegreesPerAbsEncTick = 60.0 / 1600.0;

void TurretSubsystem::Periodic()
{
    //double db = frc::SmartDashboard::GetNumber("TurretDeadbandPercent", kNeutralDeadband);
    //m_turretmotor.ConfigNeutralDeadband(db, kTimeout);

    frc::SmartDashboard::PutNumber("TurretAbsEnc", m_absEnc.GetValue());
    if (!m_setZero)
    {
        m_setZero = true;
        int zeroPos = 2400;//m_absEnc.GetValue();
        double angleChange = (zeroPos - m_startingPos) * kDegreesPerAbsEncTick;
printf("angle %.3f start pos %d cur pos %d pos delta %d deg per tick %.3f\n", angleChange, m_startingPos, m_absEnc.GetValue(), zeroPos - m_startingPos, kDegreesPerAbsEncTick);
        TurnToRelative(angleChange);
    }

    frc::SmartDashboard::PutNumber("D_T_CTicks", m_turretmotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("D_T_CAngle", TicksToDegrees(m_turretmotor.GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("D_T_CAngleCmd", m_currentAngle);
    // frc::SmartDashboard::PutNumber("D_T_DAngle", TicksToDegrees(m_turretmotor.GetClosedLoopTarget()));
    // frc::SmartDashboard::PutNumber("D_T_Error", TicksToDegrees(m_turretmotor.GetClosedLoopError(0)));
    // frc::SmartDashboard::PutNumber("D_T_Output", m_turretmotor.GetMotorOutputPercent());
    //m_turretmotor.Set(ControlMode::Position, DegreesToTicks(m_currentAngle));
    
    //m_turretmotor.Set(ControlMode::Position, 0.0);
    frc::SmartDashboard::PutNumber("ClosedLoopError", m_turretmotor.GetClosedLoopError());
    frc::SmartDashboard::PutNumber("IntegralAccumulator", m_turretmotor.GetIntegralAccumulator());

#ifdef TUNE_TURRET_PID
    double f = frc::SmartDashboard::GetNumber("TurretF", 0);
    double p = frc::SmartDashboard::GetNumber("TurretP", 0);
    double i = frc::SmartDashboard::GetNumber("TurretI", 0);
    double d = frc::SmartDashboard::GetNumber("TurretD", 0);
    m_turretmotor.Config_kF(0, f, kTimeout);
    m_turretmotor.Config_kP(0, p, kTimeout);
    m_turretmotor.Config_kI(0, i, kTimeout);
    m_turretmotor.Config_kD(0, d, kTimeout);

    double iZone = frc::SmartDashboard::GetNumber("TurretIzone", 0);
    double cruiseV = frc::SmartDashboard::GetNumber("TurretCruiseV", 60);
    double accel = frc::SmartDashboard::GetNumber("TurretAccel", 100);
    double sCurve = frc::SmartDashboard::GetNumber("TurretScurve", 1.0);

    m_turretmotor.Config_IntegralZone(0, iZone);
    m_turretmotor.ConfigMotionSCurveStrength(sCurve);
    m_turretmotor.ConfigMotionCruiseVelocity(DegreesToTicks(cruiseV/10), kTimeout);  // encoder ticks per 100ms 
    m_turretmotor.ConfigMotionAcceleration(DegreesToTicks(accel/10), kTimeout);     // encoder ticks per 100ms per sec

    // m_turretmotor.SetIntegralAccumulator(0.0, 0);
#endif
}

void TurretSubsystem::SetZeroAngle()
{
    m_currentAngle = 0;
    m_turretmotor.SetSelectedSensorPosition(0.0);
    m_setZero = false;
    m_startingPos = m_absEnc.GetValue();
}

void TurretSubsystem::TurnTo(double angle, double minAngle, double maxAngle)
{
    // Clamp the desired angle to the physical limits 
    if (angle >= minAngle && angle <= maxAngle)
        m_currentAngle = angle;
    else if (angle < minAngle)
        m_currentAngle = minAngle;
    else if (angle > maxAngle)
        m_currentAngle = maxAngle;
#ifdef USE_MOTION_MAGIC
    m_turretmotor.Set(ControlMode::MotionMagic, DegreesToTicks(m_currentAngle));
#else
    m_turretmotor.Set(ControlMode::Position, DegreesToTicks(m_currentAngle));
#endif
}

void TurretSubsystem::TurnToField(double desiredAngle)
{
    // safeguard
    //desiredAngle = Util::ZeroTo360Degs(desiredAngle);
    //double gyroAngle = Util::ZeroTo360Degs(m_gyro->GetHeading());
    double gyroAngle = m_gyro->GetHeading();
    // The difference between the field and robot is the desired angle to set relative to the robot
    double angle = gyroAngle - desiredAngle;
    TurnTo(angle);
}

void TurretSubsystem::TurnToRelative(double angle, double minAngle, double maxAngle)
{   
    double desiredAngle = TicksToDegrees(m_turretmotor.GetSelectedSensorPosition());
    if (m_dbgLogTurns)
    {
        printf("delta angle %.3f encoder %.3f current cmd %.3f\n", angle, desiredAngle, m_currentAngle);
    }
    desiredAngle += angle;
    TurnTo(desiredAngle, minAngle, maxAngle);
}

bool TurretSubsystem::isAtSetpoint()
{
    return fabs(m_turretmotor.GetClosedLoopError()) <= DegreesToTicks(kDegreeStopRange);
}

double TurretSubsystem::TicksToDegrees(double ticks)
{
    // double rev = ticks / kTicksPerRev;
    // double turretrev = rev * kMotorRevPerRev;
    // return turretrev * kDegreesPerRev;
    //return ticks * 90.0 / 6606.0;
    return ticks * 90.0 / 7700.0;
}


double TurretSubsystem::DegreesToTicks(double degrees)
{
    // double turretrev = degrees / kDegreesPerRev;
    // double rev = turretrev / kMotorRevPerRev;
    // return rev * kTicksPerRev;
    //return degrees * 6606.0 / 90.0;    // Empirically measured 6606 ticks in 90 degree swing of turret
    return degrees * 7700.0 / 90.0;    // Empirically measured 6606 ticks in 90 degree swing of turret
}

// double TurretSubsystem::GetCurrentComandedAngle()
// {
//     return m_currentAngle;
// }

double TurretSubsystem::GetCurrentAngle()
{
    return TicksToDegrees(m_turretmotor.GetSelectedSensorPosition(0));
}