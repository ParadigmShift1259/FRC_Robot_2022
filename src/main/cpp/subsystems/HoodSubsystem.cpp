#include "subsystems/HoodSubsystem.h"

#include "Constants.h"
#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <iostream>

using namespace HoodConstants;
using namespace std;
using namespace frc;

HoodSubsystem::HoodSubsystem() : m_servo(kPWMPort) {
    //SmartDashboard::PutNumber("T_H_SetAngle", 0.5);
}

void HoodSubsystem::Periodic()
{
    SmartDashboard::PutNumber("D_H_Angle", m_servo.Get());
    //double setValue = SmartDashboard::GetNumber("T_H_SetAngle",0.5);// kMax);
    //Set(setValue);
}

void HoodSubsystem::Set(double position) 
{
    // std::cout << "Servo position " << position; 
    m_servo.Set(position);
}