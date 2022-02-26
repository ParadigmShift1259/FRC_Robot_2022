#include "commands/Fire.h"
#include <frc2/command/InstantCommand.h>
#include "Constants.h"

Fire::Fire( frc::XboxController* controller
          , FlywheelSubsystem* flywheel
          , TurretSubsystem* turret
          , HoodSubsystem* hood
          , TransferSubsystem* transfer
          , VisionSubsystem& vision
          , bool* turretready
          , bool* firing
          , bool* finished
          , double launchtime)
  : m_controller(controller)
  , m_flywheel(flywheel)
  , m_turretready(turretready)
  , m_firing(firing)
  , m_finished(finished)
{
  // printf("Secondary Y firing\n");

  // Both two run parallel, but the second has a delay hack through boolean pointers
  AddCommands(
    // Home flywheel, turret, and hood to the correct speeds based on tuned fit function
      HomeTarget(m_controller, flywheel, turret, hood, vision, m_turretready, m_firing, m_finished)
    // If m_transfer ready and turret ready are true, transfer launch drives all of the balls through
    // m_turretready is checked every loop until success
    , TransferFire(transfer, m_turretready, m_firing, m_finished, launchtime)
    //, frc2::InstantCommand([this]() { m_flywheel->SetRPM(FlywheelConstants::kIdleRPM); }, {m_flywheel} )
  );
}

// void Fire::Execute()
// {
//    if (m_controller->GetBackButton())

//    {
//      printf("Fire cancelling?\n");
//      Cancel();
//    }
// }

// void Fire::End(bool interrupted)
// {
//   printf("Fire ended?\n");
//   m_flywheel->SetRPM(FlywheelConstants::kIdleRPM);
// }
