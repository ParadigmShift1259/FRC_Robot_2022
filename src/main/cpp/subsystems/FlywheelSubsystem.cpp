
#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace FlywheelConstants;

// Removes deprecated warning for CANEncoder and CANPIDController
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

FlywheelSubsystem::FlywheelSubsystem() 
    : m_flywheelmotor(kPrimaryMotorPort, CANSparkMax::MotorType::kBrushless)
    , m_flywheelFF(
        kS * 1_V, 
        FlywheelConstants::kV * 1_V * 1_s / 1_m, 
        FlywheelConstants::kA * 1_V * 1_s * 1_s / 1_m
    )
    , m_rpmSetpointLog(DataLogManager::GetLog(), "/FlyWheel/Setpoint")
    , m_rpmMeasuredLog(DataLogManager::GetLog(), "/FlyWheel/MeasuredRPM")
{
    m_flywheelmotor.SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_flywheelmotor.SetClosedLoopRampRate(0.0);
    m_flywheelmotor.SetInverted(true);

    m_flywheelPID.SetP(kP, 0);
    m_flywheelPID.SetI(kI, 0);
    m_flywheelPID.SetD(kD, 0);

    m_flywheelPID.SetP(kMP, 1);
    m_flywheelPID.SetI(kMI, 1);
    m_flywheelPID.SetD(kMD, 1);
    
    m_flywheelPID.SetOutputRange(kMinOut, kMaxOut);

    //m_flywheelencoder.SetVelocityConversionFactor(kWheelRevPerMotorRev);
    m_flywheelencoder.SetVelocityConversionFactor(1.0);

    m_setpoint = kIdleRPM / kGearRatio;

    SmartDashboard::PutNumber("T_F_Setpoint", m_setpoint);
    
    //SmartDashboard::PutNumber("FeedForward", 4.0);// SEC temp Mar 23 persistent 100 rpm overage
    SmartDashboard::PutNumber("FwAllowedErr", m_allowedError);

//Enable to tune the flywheel constants
// #define TUNE_FLYWHEEL
#ifdef TUNE_FLYWHEEL
    SmartDashboard::PutNumber("T_F_S", kS);
    SmartDashboard::PutNumber("T_F_V", FlywheelConstants::kV);
    SmartDashboard::PutNumber("T_F_A", FlywheelConstants::kA);
    SmartDashboard::PutNumber("T_F_P", kP);
    SmartDashboard::PutNumber("T_F_I", kI);
    SmartDashboard::PutNumber("T_F_D", kD);
#endif
}

#pragma GCC diagnostic pop

void FlywheelSubsystem::Periodic()
{
#ifdef TUNE_FLYWHEEL
    //double s = SmartDashboard::GetNumber("T_F_S", 0);
    //double v = SmartDashboard::GetNumber("T_F_V", 0);
    //double a = SmartDashboard::GetNumber("T_F_A", 0);
    double p = SmartDashboard::GetNumber("T_F_P", 0);
    double i = SmartDashboard::GetNumber("T_F_I", 0);
    double d = SmartDashboard::GetNumber("T_F_D", 0);
    //m_flywheelFF.kS = s * 1_V;
    //m_flywheelFF.kV = v * 1_V * 1_s / 1_m;
    //m_flywheelFF.kA = a * 1_V * 1_s * 1_s / 1_m;
    m_flywheelPID.SetP(p, 0);
    m_flywheelPID.SetI(i, 0);
    m_flywheelPID.SetD(d, 0);

    m_setpoint = SmartDashboard::GetNumber("T_F_Setpoint", 0.0);
    SmartDashboard::PutNumber("T_F_SetpointReflect", m_setpoint);
    SmartDashboard::PutNumber("T_F_Preflect", p);
    SmartDashboard::PutNumber("T_F_Ireflect", i);
    SmartDashboard::PutNumber("T_F_Dreflect", d);
#else
    SmartDashboard::PutNumber("T_F_Setpoint", m_setpoint);
#endif

    SmartDashboard::PutNumber("D_F_RPM", m_flywheelencoder.GetVelocity());
    // SmartDashboard::PutNumber("T_F_At_Target", IsAtRPM());

    m_allowedError = SmartDashboard::GetNumber("FwAllowedErr", m_allowedError);
    SmartDashboard::PutNumber("FwAllowedErrReflect", m_allowedError);

    CalculateRPM();
}

void FlywheelSubsystem::SetRPM(double setpoint)
{
    m_setpoint = setpoint / FlywheelConstants::kGearRatio;
    m_rpmSetpointLog.Append(m_setpoint);
    bool bSupressFlywheel = SmartDashboard::GetBoolean("SupressFlywheel", false);
    if (bSupressFlywheel)
    {
        m_setpoint = 0.0;
    }
    m_flywheelPID.SetIAccum(0);
}

bool FlywheelSubsystem::IsAtMaintainPID()
{
    double rpm = m_flywheelencoder.GetVelocity();
    m_error = rpm - m_setpoint;
    m_rpmMeasuredLog.Append(rpm);
    return fabs(m_error) <= kMaintainPIDError;
}

bool FlywheelSubsystem::IsAtRPM()
{
    double rpm = m_flywheelencoder.GetVelocity();
    m_error = rpm - m_setpoint;
    m_rpmMeasuredLog.Append(rpm);
    frc::SmartDashboard::PutNumber("Flywheel error", m_error);
    frc::SmartDashboard::PutNumber("Flywheel error abs", fabs(m_error));
    return fabs(m_error) <= m_allowedError;
}

bool FlywheelSubsystem::IsAtRPMPositive()
{
    double rpm = m_flywheelencoder.GetVelocity();
    m_rpmMeasuredLog.Append(rpm);
    m_error = rpm - m_setpoint;
    frc::SmartDashboard::PutNumber("Flywheel error", m_error);
    frc::SmartDashboard::PutNumber("Flywheel error abs", fabs(m_error));
    // If error is negative, always return false
    // RPM must be greater than the error with variance of Allowed Error
    return signbit(m_error) ? false : (m_error <= m_allowedError);
}

void FlywheelSubsystem::CalculateRPM()
{
    //double FF = m_setpoint * 5.0 / 2400.0 + 0.317;    // 5 / 2400 = 2.0833e-3
    double FF = m_setpoint * 2.01e-3 + 0.302;
    constexpr int pidslot = 0;
    bool bSupressFlywheel = SmartDashboard::GetBoolean("SupressFlywheel", false);

    if (bSupressFlywheel)
    {
        FF = 0.0;
    }

    SmartDashboard::PutNumber("FeedForward", FF);
    //FF = SmartDashboard::GetNumber("FeedForward", FF);
    m_flywheelPID.SetReference(m_setpoint, CANSparkMax::ControlType::kVelocity, pidslot, FF);
    m_error = m_flywheelencoder.GetVelocity()  - m_setpoint;
    frc::SmartDashboard::PutNumber("Flywheel error", m_error);
    frc::SmartDashboard::PutNumber("Flywheel error abs", fabs(m_error));

#ifdef COMMAND_TIMING
    auto endTime = m_timer.GetFPGATimestamp().to<double>();
    printf("timestamp %.3f rpm %.3f\n", endTime, m_flywheelencoder.GetVelocity());
#endif
}
