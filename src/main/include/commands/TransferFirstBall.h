#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/TransportSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class TransferFirstBall : public frc2::CommandHelper<frc2::CommandBase, TransferFirstBall> {
 public:
  explicit TransferFirstBall(TransportSubsystem* transport);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
 
 private:
  IntakeSubsystem* m_intake;
  TransportSubsystem* m_transport;
};