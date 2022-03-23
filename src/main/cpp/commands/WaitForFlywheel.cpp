
#include "commands/WaitForFlywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace IntakeConstants;

WaitForFlywheel::WaitForFlywheel(FlywheelSubsystem* flywheel)
 : m_flywheel(flywheel)
{
  AddRequirements({flywheel});
}

bool WaitForFlywheel::IsFinished()
{
  SmartDashboard::PutBoolean("D_FIRE_AT_RPM", m_flywheel->IsAtRPM());
  return m_flywheel->IsAtRPM();
}
