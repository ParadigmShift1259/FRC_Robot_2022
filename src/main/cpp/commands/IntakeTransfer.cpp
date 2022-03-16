#include "commands/IntakeTransfer.h"

#include "commands/TransferFirstBall.h"
#include "commands/TransferSecondBall.h"
#include "commands/IntakeIngest.h"
#include "commands/IntakeDeploy.h"
#include "commands/IntakeStop.h"

#include <frc2/command/WaitCommand.h>


IntakeTransfer::IntakeTransfer(ISubsysAccess& subSysAccess, double speed)
{
  AddCommands(
      IntakeDeploy(subSysAccess.GetIntake())
    , frc2::WaitCommand(0.5_s)
    , IntakeIngest(subSysAccess.GetIntake())
    , TransferFirstBall(subSysAccess.GetTransfer(), subSysAccess.GetIntake(), speed)
    , TransferSecondBall(subSysAccess)
    , IntakeStop(subSysAccess)
  );
}
