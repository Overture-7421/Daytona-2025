// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Intake/Intake.h"

struct IntakeConstants {
public:

    constexpr static const units::volt_t CoralGrab = 6.0_V;
    constexpr static const units::volt_t CoralRelease = -6.0_V;
    constexpr static const units::volt_t AlgeaGrab = 6.0_V;
    constexpr static const units::volt_t AlgeaRelease = -6.0_V;

    constexpr static const units::volt_t SlowIntake = 3.0_V;
    constexpr static const units::volt_t StopIntake = 0.0_V;
    constexpr static const units::volt_t ReverseVolts = -6.0_V;

    constexpr static const OverTalonFXConfig IntakeConfig() {
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = 1;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;

        return intakeConfig;
    }

};
