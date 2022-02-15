#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/TransportSubsystem.h"

class TransferSecondBall : public frc2::CommandHelper<frc2::CommandBase, TransferSecondBall> {
 public:
  explicit TransferSecondBall(TransportSubsystem* subsystem);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
 
 private:
  TransportSubsystem* m_transport;
  int m_photoeyeCount;
};