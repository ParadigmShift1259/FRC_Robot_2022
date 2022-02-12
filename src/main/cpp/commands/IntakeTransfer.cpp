#include "commands/IntakeTransfer.h"
#include "Constants.h"

#include "commands/TransferFirstBall.h"
#include "commands/TransferSecondBall.h"
#include "commands/IntakeIngest.h"

IntakeTransfer::IntakeTransfer(IntakeSubsystem* intake, TransferSubsystem* transfer, double speed)
{
  AddCommands(
    // Running intake
    IntakeIngest(intake),
    // Move ball to photoeye
    TransferFirstBall(transfer, speed),
    TransferSecondBall(transfer, speed)
  );
}