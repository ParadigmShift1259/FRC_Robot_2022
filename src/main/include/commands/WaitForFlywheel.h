#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FlywheelSubsystem.h"

#include "Constants.h"

class WaitForFlywheel : public frc2::CommandHelper<frc2::CommandBase, WaitForFlywheel> {
 public:
  explicit WaitForFlywheel(FlywheelSubsystem* flywheel);

  bool IsFinished() override;

 private:
  FlywheelSubsystem* m_flywheel;
};