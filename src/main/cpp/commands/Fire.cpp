#include "commands/Fire.h"
#include "Constants.h"

Fire::Fire(frc::XboxController* controller, FlywheelSubsystem* flywheel, TurretSubsystem* turret, HoodSubsystem* hood, 
            IntakeSubsystem* intake, TransferSubsystem* transfer,
            bool* turretready, bool* firing, bool* finished,
             double distance, double launchtime)
: m_controller(controller)
, m_turretready(turretready)
, m_firing(firing)
, m_finished(finished)
{
  // Both two run parallel, but the second has a delay hack through boolean pointers
  AddCommands(
    // Home flywheel, turret, and hood to the correct speeds based on tuned fit function
    HomeTarget(m_controller, flywheel, turret, hood, m_turretready, m_firing, m_finished, distance),
    // If m_transfer ready and turret ready are true, transfer launch drives all of the balls through
    // m_turretready is checked every loop until success
    TransferFire(transfer, m_turretready, m_firing, m_finished, launchtime)
  );
}