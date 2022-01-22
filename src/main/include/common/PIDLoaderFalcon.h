#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include <string>

using namespace frc;
using namespace std;
using namespace ctre::phoenix::motorcontrol::can;


class PIDLoaderFalcon
{
public:
    PIDLoaderFalcon(string name, bool adjustable, double p, double i, double d, double ff);
    PIDLoaderFalcon(string name, bool adjustable, double p, double i, double d, double ff, double max, double min);

    /// Loads drive PID controller with values, also sends default PID values to SmartDashboard
    /// \param driveMotor        The TalonFX responsible for driving
    void Load(TalonFX& driveMotor);

    /// Loads drive PID controller with on the fly values from SmartDashboard
    /// \param driveMotor        The TalonFX responsible for driving
    void LoadFromNetworkTable(TalonFX& driveMotor);

private:
    string m_name;
    bool m_adjustable;

    double m_p;
    double m_i;
    double m_d;
    double m_ff;
    double m_max = 1.0;
    double m_min = -1.0;
};