
#include "subsystems/IntakeSubsystem.h"

#include "ConstantsIntake.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace IntakeConstants;
using namespace std;
using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kMotorPort)
{

    m_motor.SetNeutralMode(NeutralMode::Coast);

}

void IntakeSubsystem::Periodic()
{
    // SmartDashboard::PutNumber("D_I_Motor", m_motor.Get());
}

void IntakeSubsystem::Set(double speed) 
{
    m_motor.Set(ControlMode::PercentOutput, speed * kMotorReverseConstant);
}