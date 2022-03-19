
#include "commands/WaitForFlywheel.h"

#include "Constants.h"

using namespace IntakeConstants;

WaitForFlywheel::WaitForFlywheel(FlywheelSubsystem* flywheel)
 : m_flywheel(flywheel)
{
  AddRequirements({flywheel});
}

bool WaitForFlywheel::IsFinished()
{
  return m_flywheel->IsAtRPM();
}
