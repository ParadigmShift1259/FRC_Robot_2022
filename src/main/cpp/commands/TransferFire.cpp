#include "commands/TransferFire.h"
#include "Constants.h"

TransferFire::TransferFire(TransferSubsystem* transfer, 
                        bool* turretready, bool* firing, bool* finished, 
                        double launchtime)
{
    AddCommands(
        TransferPrepare(transfer, false),
        TransferLaunch(transfer, turretready, firing, finished, launchtime),
    );
}