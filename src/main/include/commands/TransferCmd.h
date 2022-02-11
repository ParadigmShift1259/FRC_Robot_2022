#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/TransferSubsystem.h"

///#include "Constants.h"

class TransferCmd : public frc2::CommandHelper<frc2::CommandBase, TransferCmd> {
 public:
  explicit TransferCmd(TransferSubsystem* subsystem, double speed);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
 
 private:
  TransferSubsystem* m_transfer;
  frc::Timer m_timer;
  double m_speed;
};