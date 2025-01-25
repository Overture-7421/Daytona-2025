// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <Subsystems/Arm/Arm.h>
#include <Subsystems/Elevator/Elevator.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ArmHighMotion
    : public frc2::CommandHelper<frc2::Command, ArmHighMotion> {
 public:
 
  ArmHighMotion(Elevator* elevator, Arm* arm, units::degree_t armAngle, units::degree_t wristAngle, units::meter_t elevatorPosition );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private: 

  Elevator* elevator;
  Arm* arm;
  units::degree_t armAngle;
  units::degree_t wristAngle;
  units::meter_t elevatorPosition;

};
