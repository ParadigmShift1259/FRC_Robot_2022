
#include "commands/IntakeRelease.h"

#include "Constants.h"

using namespace IntakeConstants;

IntakeRelease::IntakeRelease(ISubsysAccess& subsytemAccess)
 : m_intake(subsytemAccess.GetIntake())
 , m_transfer(subsytemAccess.GetTransfer())
{
  AddRequirements({&subsytemAccess.GetIntake(), &subsytemAccess.GetTransfer()});
}

void IntakeRelease::Execute() {
  m_intake.Set(kReleaseHigh);
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  m_transfer.SetFeeder(0);
  m_transfer.SetTransfer(0);
}
