#include "common/PIDLoaderFalcon.h"
#include <frc/SmartDashboard/SmartDashboard.h>

using namespace frc;

PIDLoaderFalcon::PIDLoaderFalcon(string name, bool adjustable, double p, double i, double d, double ff)
    : m_name(name)
    , m_adjustable(adjustable)
    , m_p(p)
    , m_i(i)
    , m_d(d)
    , m_ff(ff) {}

PIDLoaderFalcon::PIDLoaderFalcon(string name, bool adjustable, double p, double i, double d, double ff, double max, double min)
    : m_name(name)
    , m_adjustable(adjustable)
    , m_p(p)
    , m_i(i)
    , m_d(d)
    , m_ff(ff)
    , m_max(max)
    , m_min(min) {}

void PIDLoaderFalcon::Load(TalonFX& driveMotor)
{
    driveMotor.Config_kP(0, m_p);
    driveMotor.Config_kD(0, m_d);
    driveMotor.Config_kF(0, m_ff);
    driveMotor.ConfigPeakOutputForward(m_max);
    driveMotor.ConfigPeakOutputReverse(m_min);

    if (m_adjustable)
    {
        // SmartDashboard::PutNumber("T_" + m_name + "_DP", m_p);
        // SmartDashboard::PutNumber("T_" + m_name + "_DI", m_i);
        // SmartDashboard::PutNumber("T_" + m_name + "_DD", m_d);
        // SmartDashboard::PutNumber("T_" + m_name + "_DFF", m_ff);
        // SmartDashboard::PutNumber("T_" + m_name + "_DMax", m_max);
        // SmartDashboard::PutNumber("T_" + m_name + "_DMin", m_min);
    }
}

void PIDLoaderFalcon::LoadFromNetworkTable(TalonFX& driveMotor)
{
    if (!m_adjustable)
        return;

    double p = SmartDashboard::GetNumber("T_" + m_name + "_DP", m_p);
    double i = SmartDashboard::GetNumber("T_" + m_name + "_DI", m_i);
    double d = SmartDashboard::GetNumber("T_" + m_name + "_DD", m_d);
    double ff = SmartDashboard::GetNumber("T_" + m_name + "_DFF", m_ff);
    double max = SmartDashboard::GetNumber("T_" + m_name + "_DMax", m_max);
    double min = SmartDashboard::GetNumber("T_" + m_name + "_DMin", m_min);

    /// if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_p)) { driveMotor.Config_kP(0, p); m_p = p; }
    if ((i != m_i)) { driveMotor.Config_kI(0, i); m_i = i; }
    if ((d != m_d)) { driveMotor.Config_kD(0, d); m_d = d; }
    if ((ff != m_ff)) { driveMotor.Config_kF(0, ff); m_ff = ff; }
    
    if ((max != m_max) || (min != m_min))
    {
        driveMotor.ConfigPeakOutputForward(m_max);
        driveMotor.ConfigPeakOutputReverse(m_min);
        m_min = min;
        m_max = max; 
    }
}