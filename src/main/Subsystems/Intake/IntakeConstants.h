// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Intake/Intake.h"

struct IntakeConstants {
public:

    constexpr static const units::volt_t CoralGrab = 4.0_V;
    constexpr static const units::volt_t CoralRelease = -4.0_V;
    constexpr static const units::volt_t AlgeaGrab = 4.0_V;
    constexpr static const units::volt_t AlgeaRelease = -4.0_V;

    constexpr static const units::volt_t SlowIntake = 3.0_V;
    constexpr static const units::volt_t StopIntake = 0.0_V;
    constexpr static const units::volt_t ReverseVolts = -4.0_V;

    constexpr static const OverTalonFXConfig IntakeConfig() {
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = 54;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeConfig.Inverted = true;

        intakeConfig.CurrentLimit = 40_A;
        intakeConfig.StatorCurrentLimit = 120_A;
        intakeConfig.TriggerThreshold = 60_A;
        intakeConfig.TriggerThresholdTime = 1_s;
        intakeConfig.ClosedLoopRampRate = 0.0_s;
        intakeConfig.OpenLoopRampRate = 0.01_s;

        return intakeConfig;
    }

};
