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
//#define FIRE_WITH_TIMED_DELAY_TO_SECOND_BALL
  AddCommands(
#ifdef FIRE_WITH_TIMED_DELAY_TO_SECOND_BALL
      HomeTarget(flywheel, turret, hood, vision, m_turretready, m_firing, m_finished, yVelocityCb)
    , TransferFire(transfer, m_turretready, m_firing, m_finished, launchtime)
#else
      HomeTarget(flywheel, turret, hood, vision, m_turretready, m_firing, m_finished, yVelocityCb)
#ifdef SAVE
    , FireOneBall(transfer)
#else
    , FireOneBall(subsystemAccess)
#endif
    , TransferFirstBall(transfer)
    , WaitForFlywheel(flywheel)
#ifdef SAVE
    , FireOneBall(transfer)
#else
    , FireOneBall(subsystemAccess)
#endif
#endif
  );
}
