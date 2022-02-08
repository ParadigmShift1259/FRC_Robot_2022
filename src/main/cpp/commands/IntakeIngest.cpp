
#include "commands/IntakeIngest.h"
#include "Constants.h"

using namespace IntakeConstants;

IntakeIngest::IntakeIngest(IntakeSubsystem* subsystem) 
: m_intake(subsystem)
{
  // printf("Initailized Intake Ingest command");
  AddRequirements({subsystem});
  
}

void IntakeIngest::Execute() {
    m_intake->Set(kIngestHigh);
    m_bRunning = true;
}

bool IntakeIngest::IsFinished()
{
  return m_bRunning;
}

void IntakeIngest::End(bool interrupted) {
    m_intake->Set(0);
    m_bRunning = false;
}