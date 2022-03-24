
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
    m_turretmotor.Config_IntegralZone(0, kIZone, kTimeout);
    m_turretmotor.ConfigMaxIntegralAccumulator(0, 50000.0); // TO DO: remove or tune
    m_turretmotor.SetIntegralAccumulator(0.0, 0);
    m_turretmotor.ConfigMotionSCurveStrength(kMotionSCurveStrength);

    // set soft limits of turret
    m_turretmotor.ConfigForwardSoftLimitThreshold(DegreesToTicks(kMaxAngle));
    m_turretmotor.ConfigReverseSoftLimitThreshold(DegreesToTicks(kMinAngle));
    m_turretmotor.ConfigForwardSoftLimitEnable(true);
    m_turretmotor.ConfigReverseSoftLimitEnable(true);

    //m_turretmotor.ConfigNeutralDeadband(kNeutralDeadband, kTimeout); // TO DO: remove or tune
    //m_turretmotor.ConfigNominalOutputForward(0.0);
    //m_turretmotor.ConfigNominalOutputReverse(0.0);
    m_turretmotor.ConfigPeakOutputForward(kMaxOut, kTimeout);
    m_turretmotor.ConfigPeakOutputReverse(kMaxOut * -1.0, kTimeout);
    //m_turretmotor.ConfigAllowableClosedloopError(0, DegreesToTicks(kDegreePIDStopRange), kTimeout);
    m_turretmotor.ConfigMotionCruiseVelocity(DegreesToTicks(kMMCruiseVel/10), kTimeout);  // encoder ticks per 100ms 
    m_turretmotor.ConfigMotionAcceleration(DegreesToTicks(kMMAccel/10), kTimeout);     // encoder ticks per 100ms per sec

#define USE_MOTION_MAGIC
#ifdef USE_MOTION_MAGIC
    m_turretmotor.Set(ControlMode::MotionMagic, DegreesToTicks(kStartingPositionDegrees));
#else
    m_turretmotor.Set(ControlMode::Position, DegreesToTicks(kStartingPositionDegrees));
#endif
    //m_turretmotor.Set(ControlMode::PercentOutput, 0.1);
    m_currentAngle = kStartingPositionDegrees;
    m_startingPos = GetAbsEncValue();
    frc::SmartDashboard::PutNumber("TurretStartingPos", m_startingPos);

#ifdef TUNE_TURRET_PID
    frc::SmartDashboard::PutNumber("TurretP", kP);
    frc::SmartDashboard::PutNumber("TurretI", kI);
    frc::SmartDashboard::PutNumber("TurretD", kD);
    frc::SmartDashboard::PutNumber("TurretF", kF);

    frc::SmartDashboard::PutNumber("TurretIzone", kIZone);
    frc::SmartDashboard::PutNumber("TurretCruiseV", kMMCruiseVel);
    frc::SmartDashboard::PutNumber("TurretAccel", kMMAccel);
    frc::SmartDashboard::PutNumber("TurretScurve", 1.0);
#endif

    //frc::SmartDashboard::PutNumber("TurretDeadbandPercent", kNeutralDeadband);
}

//constexpr double kDegreesPerAbsEncTick = 1 / 24.6;//60.0 / 1600.0;

void TurretSubsystem::Periodic()
{
    //double db = frc::SmartDashboard::GetNumber("TurretDeadbandPercent", kNeutralDeadband);
    //m_turretmotor.ConfigNeutralDeadband(db, kTimeout);

    if (!m_setZero)
    {
        double CTREpos = (m_startingPos - kAbsEncoderZero) * kCtreTicksPerAbsEncTick;
        printf("start abs %d cur abs %d abs delta %d ctre ticks %.3f ctre tick per abs tick %.3f\n", m_startingPos, GetAbsEncValue(), m_startingPos - kAbsEncoderZero, CTREpos, kCtreTicksPerAbsEncTick);
        m_turretmotor.SetSelectedSensorPosition(CTREpos);
        m_currentAngle = GetCurrentAngle();
        m_setZero = true;
        TurnTo(0.0);
    }

    frc::SmartDashboard::PutNumber("D_T_CAbsEncoder", GetAbsEncValue());
    frc::SmartDashboard::PutNumber("D_T_CTicks", m_turretmotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("D_T_CAngle", TicksToDegrees(m_turretmotor.GetSelectedSensorPosition()));
    frc::SmartDashboard::PutNumber("D_T_CVelocity", m_turretmotor.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("D_T_CAngleCmd", m_currentAngle);
    frc::SmartDashboard::PutNumber("D_T_COutput", m_turretmotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("D_T_CClosedLoopError", m_turretmotor.GetClosedLoopError());
    frc::SmartDashboard::PutNumber("D_T_CIntegralAccumulator", m_turretmotor.GetIntegralAccumulator());
    // frc::SmartDashboard::PutNumber("D_T_DAngle", TicksToDegrees(m_turretmotor.GetClosedLoopTarget()));
    // frc::SmartDashboard::PutNumber("D_T_Error", TicksToDegrees(m_turretmotor.GetClosedLoopError(0)));
    // frc::SmartDashboard::PutNumber("D_T_Output", m_turretmotor.GetMotorOutputPercent());
#ifdef TUNE_TURRET_PID
    
    static double lastF;
    static double lastI;
    static double lastP;
    static double lastD;
    static double lastIZone;
    static double lastCruiseV;
    static double lastAccel;
    static double lastSCurve;

    double f = frc::SmartDashboard::GetNumber("TurretF", 0);
    double p = frc::SmartDashboard::GetNumber("TurretP", 0);
    double i = frc::SmartDashboard::GetNumber("TurretI", 0);
    double d = frc::SmartDashboard::GetNumber("TurretD", 0);
    double iZone = frc::SmartDashboard::GetNumber("TurretIzone", 0);
    double cruiseV = frc::SmartDashboard::GetNumber("TurretCruiseV", 60);
    double accel = frc::SmartDashboard::GetNumber("TurretAccel", 100);
    double sCurve = frc::SmartDashboard::GetNumber("TurretScurve", 1.0);

    if (f != lastF || p != lastP || i != lastI || d != lastD || 
    iZone != lastIZone || cruiseV != lastCruiseV || accel != lastAccel || sCurve != lastSCurve) 
        {
        m_turretmotor.Config_kF(0, f, kTimeout);
        m_turretmotor.Config_kP(0, p, kTimeout);
        m_turretmotor.Config_kI(0, i, kTimeout);
        m_turretmotor.Config_kD(0, d, kTimeout);
        m_turretmotor.Config_IntegralZone(0, iZone);
        m_turretmotor.ConfigMotionSCurveStrength(sCurve);
        m_turretmotor.ConfigMotionCruiseVelocity(DegreesToTicks(cruiseV/10), kTimeout);  // encoder ticks per 100ms 
        m_turretmotor.ConfigMotionAcceleration(DegreesToTicks(accel/10), kTimeout);     // encoder ticks per 100ms per sec
        }

     lastF = f;
     lastP = p;
     lastI = i;
     lastD = d;
     lastIZone = iZone;
     lastAccel = accel;
     lastCruiseV = cruiseV;
     lastSCurve = sCurve;



    // m_turretmotor.SetIntegralAccumulator(0.0, 0);

#else
    // m_turretmotor.Config_kF(0, kF, kTimeout);
    // m_turretmotor.Config_kP(0, kP, kTimeout);
    // m_turretmotor.Config_kI(0, kI, kTimeout);
    // m_turretmotor.Config_kD(0, kD, kTimeout);

    // m_turretmotor.Config_IntegralZone(0, 1000.0);
    // m_turretmotor.ConfigMotionSCurveStrength(1.0);
    // m_turretmotor.ConfigMotionCruiseVelocity(DegreesToTicks(kMMCruiseVel / 10.0), kTimeout);  // encoder ticks per 100ms 
    // m_turretmotor.ConfigMotionAcceleration(DegreesToTicks(kMMAccel / 10.0), kTimeout);     // encoder ticks per 100ms per sec
#endif
}

void TurretSubsystem::SetZeroAngle()
{
    //m_currentAngle = 0;
    //m_turretmotor.SetSelectedSensorPosition(0.0);
    m_setZero = false;
    m_startingPos = GetAbsEncValue();
}

void TurretSubsystem::TurnTo(double angle, double minAngle, double maxAngle)
{
    if (m_setZero)
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
    return ticks / kTicksPerDegree;
}

double TurretSubsystem::DegreesToTicks(double degrees)
{
    return degrees * kTicksPerDegree;
}

// double TurretSubsystem::GetCurrentComandedAngle()
// {
//     return m_currentAngle;
// }

double TurretSubsystem::GetCurrentAngle()
{
    return TicksToDegrees(m_turretmotor.GetSelectedSensorPosition(0));
}