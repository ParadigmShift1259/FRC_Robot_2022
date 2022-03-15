
#include "commands/IntakeDeploy.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeDeploy::IntakeDeploy(IntakeSubsystem& subsystem) 
  : m_intake(subsystem)
{
  AddRequirements({&subsystem});
}

void IntakeDeploy::Execute() {
    m_intake.IntakeOut(true);
    m_bRunning = true;
}

bool IntakeDeploy::IsFinished()
{
  return m_bRunning;
}

void IntakeDeploy::End(bool interrupted) {
    m_bRunning = false;
}