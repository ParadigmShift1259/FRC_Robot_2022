#include "commands/IntakeTransfer.h"
#include "Constants.h"

#include "commands/TransferFirstBall.h"
#include "commands/TransferSecondBall.h"
#include "commands/IntakeIngest.h"

IntakeTransfer::IntakeTransfer(IntakeSubsystem* intake, TransportSubsystem* transport)
{
  AddCommands(
    
      IntakeIngest(intake)            // Running intake
    , TransferFirstBall(transport)    // Move ball to feeder photoeye
    , TransferSecondBall(transport)   // Move ball to transfer photoeye
  );
}