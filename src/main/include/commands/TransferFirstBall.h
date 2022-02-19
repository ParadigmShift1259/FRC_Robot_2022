#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TransferSubsystem.h"

class TransferFirstBall : public frc2::CommandHelper<frc2::CommandBase, TransferFirstBall> {
 public:
  explicit TransferFirstBall(TransferSubsystem& transfer, IntakeSubsystem& intake, double speed);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
 
 private:
  IntakeSubsystem& m_intake;
  TransferSubsystem& m_transfer;
  double m_speed;
};