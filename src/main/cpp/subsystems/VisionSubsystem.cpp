
#include "subsystems/VisionSubsystem.h"


VisionSubsystem::VisionSubsystem() 
 : m_dashboard (nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard"))
 , m_networktable(nt::NetworkTableInstance::GetDefault().GetTable("limelight"))
 , m_led(true)
 , m_tx(0)
 , m_ty(0)
 , m_validTarget(false)
{
    SetLED(true);
    m_averageDistance.reserve(3);
    m_averageAngle.reserve(3);
}

void VisionSubsystem::Periodic()
{
    m_validTarget = m_networktable->GetNumber("tv", 0);

    if (!m_led)
        m_validTarget = false;

    m_tx = m_networktable->GetNumber("tx", 0.0);
    m_ty = m_networktable->GetNumber("ty", 0.0);

    double verticalAngle = kMountingAngle + m_ty;    
    double horizontalAngle = m_tx;  // in degrees
    
    double distance = (kTargetHeight - kMountingHeight) / tanf(Util::DegreesToRadians(verticalAngle));
    
    if ((distance < kMinTargetDistance) || (distance > kMaxTargetDistance))
        m_validTarget = false;

    if (!m_validTarget)
    {
        m_averageDistance.clear();
        m_averageAngle.clear();
        return;
    }

    m_averageDistance.push_back(distance);
    m_averageAngle.push_back(horizontalAngle);

    if (m_averageDistance.size() > 3)
    m_averageDistance.erase(m_averageDistance.begin());

    if (m_averageAngle.size() > 3)
        m_averageAngle.erase(m_averageAngle.begin());

    SmartDashboard::PutNumber("D_V_Active", m_validTarget);
    SmartDashboard::PutNumber("D_V_Distance", distance);
    // SmartDashboard::PutNumber("D_V_Angle", m_horizontalangle);
    SmartDashboard::PutNumber("D_V_Average Distance", GetDistance());
    SmartDashboard::PutNumber("D_V_Average Angle", GetAngle());
}

bool VisionSubsystem::GetValidTarget()
{
    return m_validTarget;
}

double VisionSubsystem::GetDistance()
{
    return Util::GetAverage(m_averageDistance);
}


double VisionSubsystem::GetAngle()
{
    return Util::GetAverage(m_averageAngle);
}

void VisionSubsystem::SetLED(bool on)
{
    m_led = on;
    if (m_led)
    {
        /// 3 forces limelight led on
        m_networktable->PutNumber("ledMode", 3);
    }
    else
    {
        /// 1 forces limelight led off
        m_networktable->PutNumber("ledMode", 1);
    }
}