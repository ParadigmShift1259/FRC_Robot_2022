
#include "commands/IntakeStop.h"

#include "Constants.h"

using namespace IntakeConstants;

IntakeStop::IntakeStop(ISubsysAccess& subsytemAccess)
 : m_intake(subsytemAccess.GetIntake())
{
  AddRequirements({&subsytemAccess.GetIntake(), &subsytemAccess.GetTransfer()});
}

void IntakeStop::Execute() {
  m_intake.Set(0);
}

bool IntakeStop::IsFinished()
{
  return true;
}

void IntakeStop::End(bool interrupted) {
  m_intake.IntakeOut(false);
}
