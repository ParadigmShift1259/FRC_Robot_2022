#include "commands/IntakeTransfer.h"

#include "commands/TransferFirstBall.h"
#include "commands/TransferSecondBall.h"
#include "commands/IntakeIngest.h"

IntakeTransfer::IntakeTransfer(ISubsysAccess& subSysAccess, double speed)
{
  AddCommands(
      IntakeIngest(subSysAccess.GetIntake())
    , TransferFirstBall(subSysAccess.GetTransfer(), subSysAccess.GetIntake(), speed)
    , TransferSecondBall(subSysAccess.GetTransfer(), subSysAccess.GetIntake(), speed)
  );
}
