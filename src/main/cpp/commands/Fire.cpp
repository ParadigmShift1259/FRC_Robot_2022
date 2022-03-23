#include "commands/Fire.h"
#include <frc2/command/InstantCommand.h>

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
      HomeTarget(flywheel, turret, hood, vision, m_turretready, m_firing, m_finished, yVelocityCb)
    , FireOneBall(transfer)
    , TransferFirstBall(transfer)
    , WaitForFlywheel(flywheel)
    , FireOneBall(transfer)
  );
}
