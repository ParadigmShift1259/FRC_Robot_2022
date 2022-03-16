
#include "commands/IntakeRelease.h"

#include "Constants.h"

using namespace IntakeConstants;

IntakeRelease::IntakeRelease(ISubsysAccess& subsytemAccess)
 : m_intake(subsytemAccess.GetIntake())
 , m_transfer(subsytemAccess.GetTransfer())
 , m_subsystemAccess(subsytemAccess)
{
  AddRequirements({&subsytemAccess.GetIntake(), &subsytemAccess.GetTransfer()});
}

void IntakeRelease::Execute() {
  m_subsystemAccess.SetOneBallFlag();
  m_intake.Set(kReleaseSpeed);
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  m_intake.IntakeOut(false);
  m_transfer.SetFeeder(0);
  m_transfer.SetTransfer(0);
}
