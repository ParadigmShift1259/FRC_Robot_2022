#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TransferSubsystem.h"

class TransferSecondBall : public frc2::CommandHelper<frc2::CommandBase, TransferSecondBall>
{
 public:
  explicit TransferSecondBall(TransferSubsystem& transfer, IntakeSubsystem& intake, double speed);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
 
 private:
  IntakeSubsystem& m_intake;
  TransferSubsystem& m_transfer;
  double m_speed;
  int m_photoeyeCount;
};