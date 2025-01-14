// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/FunctionalCommand.h>
// #include <units/angular_acceleration.h>

#include "Subsystems/Arm/Constants.h"

class Arm : public frc2::SubsystemBase
{
public:
  Arm();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  void setToAngle(units::degree_t armAngle, units::degree_t wristAngle);
  frc2::CommandPtr setArmCommand(units::degree_t armAngle, units::degree_t wristAngle);

  bool isArmAtPosition(units::degree_t armAngle, units::degree_t wristAngle);
  void getCurrentAngle(double currentArmAngle, double currentWristAngle);

  OverTalonFX armLeftMotor{Constants::armLeftConfig(), "rio"};
  OverTalonFX armRightMotor{Constants::armRightConfig(), "rio"};
  OverCANCoder armCANCoder{Constants::armCANConfig(), "rio"};
  OverTalonFX wristMotor{Constants::wristConfig(), "rio"};
  OverCANCoder wristCANCoder{Constants::wristCANConfig(), "rio"};

  void Periodic() override;

private:
  frc2::sysid::SysIdRoutine m_sysIdRoutine{frc2::sysid::Config{1_V / 1_s, 3_V, 30_s, nullptr},
    frc2::sysid::Mechanism{
        [this](units::volt_t driveVoltage)
        {
          armLeftMotor.SetVoltage(driveVoltage);
        },
        [this](frc::sysid::SysIdRoutineLog *log)
        {
          log->Motor("lowerArm")
              .voltage(armLeftMotor.GetMotorVoltage().GetValue())
              .position(armLeftMotor.GetPosition().GetValue())
              .velocity(armLeftMotor.GetVelocity().GetValue());
        },
        this}};

  frc::ArmFeedforward armFeedForward{0.0_V, 0.0_V, 0.0_V / 0_tps, 0.0_V / 0_tr_per_s_sq};
  frc::ArmFeedforward wristFeedForward{0.0_V, 0.0_V, 0.0_V / 0_tps, 0_V / 0_tr_per_s_sq};

  MotionMagicVoltage armVoltage{0_tr};
  MotionMagicVoltage wristVoltage{0_tr};
};
