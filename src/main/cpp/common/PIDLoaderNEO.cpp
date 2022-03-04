#include "common/PIDLoaderNEO.h"


PIDLoaderNEO::PIDLoaderNEO(string name, bool adjustable, double p, double i, double d, double iz, double ia)
    : m_name(name)
    , m_adjustable(adjustable)
    , m_p(p)
    , m_i(i)
    , m_d(d)
    , m_iz(iz)
    , m_ia(ia) {}

PIDLoaderNEO::PIDLoaderNEO(string name, bool adjustable, double p, double i, double d, double iz, double ia, double ff, double max, double min)
    : m_name(name)
    , m_adjustable(adjustable)
    , m_p(p)
    , m_i(i)
    , m_d(d)
    , m_iz(iz)
    , m_ia(ia)
    , m_ff(ff)
    , m_max(max)
    , m_min(min) {}

void PIDLoaderNEO::Load(SparkMaxPIDController& turnPIDController)
{
    turnPIDController.SetP(m_p);
    turnPIDController.SetI(m_i);
    turnPIDController.SetD(m_d);
    turnPIDController.SetIZone(m_iz);
    turnPIDController.SetIMaxAccum(m_ia);
    turnPIDController.SetFF(m_ff);
    turnPIDController.SetOutputRange(m_min, m_max);
    
    if (m_adjustable)
    {
        // SmartDashboard::PutNumber("T_" + m_name + "_TP", m_p);
        // SmartDashboard::PutNumber("T_" + m_name + "_TI", m_i);
        // SmartDashboard::PutNumber("T_" + m_name + "_TD", m_d);
        // SmartDashboard::PutNumber("T_" + m_name + "_TIZone", m_iz);
        // SmartDashboard::PutNumber("T_" + m_name + "_TIAccum", m_ia);
        // SmartDashboard::PutNumber("T_" + m_name + "_TFF", m_ff);
        // SmartDashboard::PutNumber("T_" + m_name + "_TMax", m_max);
        // SmartDashboard::PutNumber("T_" + m_name + "_TMin", m_min);
    }
}

void PIDLoaderNEO::LoadFromNetworkTable(SparkMaxPIDController& turnPIDController)
{
    if (!m_adjustable)
        return;

    double p = SmartDashboard::GetNumber("T_" + m_name + "_TP", m_p);
    double i = SmartDashboard::GetNumber("T_" + m_name + "_TI", m_i);
    double d = SmartDashboard::GetNumber("T_" + m_name + "_TD", m_d);
    double iz = SmartDashboard::GetNumber("T_" + m_name + "_TIZone", m_iz);
    double ia = SmartDashboard::GetNumber("T_" + m_name + "_TIAccum", m_ia);
    double ff = SmartDashboard::GetNumber("T_" + m_name + "_TFF", m_ff);
    double max = SmartDashboard::GetNumber("T_" + m_name + "_TMax", m_max);
    double min = SmartDashboard::GetNumber("T_" + m_name + "_TMin", m_min);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_p)) { turnPIDController.SetP(p); m_p = p; }
    if ((i != m_i)) { turnPIDController.SetI(i); m_i = i; }
    if ((d != m_d)) { turnPIDController.SetD(d); m_d = d; }
    if ((iz != m_iz)) { turnPIDController.SetIZone(iz); m_iz = iz; }
    if ((ia != m_ia)) { turnPIDController.SetIAccum(ia); m_ia = ia; }
    if ((ff != m_ff)) { turnPIDController.SetFF(ff); m_ff = ff; }
    
    if ((max != m_max) || (min != m_min))
    { 
        turnPIDController.SetOutputRange(min, max);
        m_min = min;
        m_max = max; 
    }
}