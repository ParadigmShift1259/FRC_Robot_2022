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
  TransferSubsystem& m_transfer;
  IntakeSubsystem& m_intake;
  double m_speed;
};