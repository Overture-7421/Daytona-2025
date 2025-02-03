// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Intake/Intake.h"

struct IntakeConstants {
public:

    constexpr static const units::degree_t JawAlgae = 100_deg;
    constexpr static const units::degree_t JawCoralOpen = 50_deg;
    constexpr static const units::degree_t JawCoralClose = 10_deg;

    constexpr static const units::volt_t CoralGrab = 4.0_V;
    constexpr static const units::volt_t CoralRelease = -4.0_V;

    constexpr static const units::volt_t AlgaeGrab = 4.0_V;
    constexpr static const units::volt_t AlgaeRelease = -4.0_V;

    constexpr static const units::volt_t SlowIntake = 3.0_V;
    constexpr static const units::volt_t StopIntake = 0.0_V;
    constexpr static const units::volt_t ReverseVolts = -4.0_V;

    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 25.5_tps;
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 35_tr_per_s_sq;

    constexpr static const double SensorToMechanism = 1.5; //Emi will tell us

    constexpr static const OverTalonFXConfig IntakeConfig() {
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = 54;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeConfig.Inverted = true;

        intakeConfig.CurrentLimit = 20_A;
        intakeConfig.StatorCurrentLimit = 120_A;
        intakeConfig.TriggerThreshold = 60_A;
        intakeConfig.TriggerThresholdTime = 1_s;
        intakeConfig.ClosedLoopRampRate = 0.0_s;
        intakeConfig.OpenLoopRampRate = 0.05_s;

        return intakeConfig;
    }

    constexpr static const OverTalonFXConfig IntakeJawConfig() {
        OverTalonFXConfig intakeJawConfig;
        intakeJawConfig.MotorId = 58;
        intakeJawConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeJawConfig.useFOC = true;
        intakeJawConfig.Inverted = true;

        intakeJawConfig.ClosedLoopRampRate = 0.05_s;
        intakeJawConfig.CurrentLimit = 30_A;
        intakeJawConfig.StatorCurrentLimit = 30_A;
        intakeJawConfig.TriggerThreshold = 90_A;
        intakeJawConfig.TriggerThresholdTime = 1_s;
        intakeJawConfig.OpenLoopRampRate = 0.05_s;
        intakeJawConfig.PIDConfigs.WithKP(4.0);

        return intakeJawConfig;
    }

};
