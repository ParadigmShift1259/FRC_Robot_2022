#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TransferSubsystem.h"

class TransferFirstBall : public frc2::CommandHelper<frc2::CommandBase, TransferFirstBall> {
 public:
  explicit TransferFirstBall(TransferSubsystem* transfer);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
 
 private:
  TransferSubsystem* m_transfer;
};