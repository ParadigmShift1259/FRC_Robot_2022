#include "commands/Fire.h"
#include <frc2/command/InstantCommand.h>
//#include <frc2/command/WaitCommand.h>

#include "Constants.h"

using namespace TransferConstants;

Fire::Fire( FlywheelSubsystem* flywheel
          , TurretSubsystem* turret
          , HoodSubsystem* hood
          , TransferSubsystem* transfer
          , VisionSubsystem& vision
          , bool* turretready
          , bool* firing
          , bool* finished
          , GetYvelocityCallback yVelocityCb
          , double launchtime)
  : m_flywheel(flywheel)
  , m_turretready(turretready)
  , m_firing(firing)
  , m_finished(finished)
{
  AddCommands(
      // Home flywheel, turret, and hood to the correct speeds based on tuned fit function
      HomeTarget(flywheel, turret, hood, vision, m_turretready, m_firing, m_finished, yVelocityCb)
      // If m_transfer ready and turret ready are true, transfer launch drives all of the balls through
      // m_turretready is checked every loop until success
    //, TransferFire(transfer, m_turretready, m_firing, m_finished, launchtime)
    , FireOneBall(transfer)
    , TransferFirstBall(transfer)
    , WaitForFlywheel(flywheel)
    //, HomeTarget(flywheel, turret, hood, vision, m_turretready, m_firing, m_finished, yVelocityCb) //TO DO: If firing takes too long check for flywheel in transferFire
    , FireOneBall(transfer)
    //, frc2::InstantCommand([this, &transfer]() { transfer->SetTransfer(0.0); }, {transfer} )
  );
}

//void Fire::Execute()
//{
//    printf("Firing\n");
//    if (m_controller->GetBackButton())

//    {
//      printf("Fire cancelling?\n");
//      Cancel();
//    }
//}

// void Fire::End(bool interrupted)
// {
//   printf("Fire ended?\n");
//   m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
// }
