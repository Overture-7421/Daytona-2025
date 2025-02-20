// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Intake/Intake.h"

struct IntakeConstants {
public:

    constexpr static const units::degree_t RangeError = 1.0_deg;

    constexpr static const units::meter_t SensorCoralDistance = 0.10_m;
    constexpr static const units::meter_t SensorAlgaeDistance = 0.10_m;

    constexpr static const units::degree_t JawAlgae = 40_deg;
    constexpr static const units::degree_t JawCoralOpen = 10_deg;
    constexpr static const units::degree_t JawCoralClose = 3_deg;

    constexpr static const units::volt_t CoralGrab = 4.0_V;
    constexpr static const units::volt_t CoralRelease = -4.0_V;

    constexpr static const units::volt_t AlgaeGrab = 4.0_V;
    constexpr static const units::volt_t AlgaeRelease = -4.0_V;

    constexpr static const units::volt_t SlowIntake = 3.0_V;
    constexpr static const units::volt_t StopIntake = 0.0_V;
    constexpr static const units::volt_t ReverseVolts = -4.0_V;

    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 15_tps; //25.5
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 12_tr_per_s_sq; //35

    constexpr static const double SensorToMechanism = 1.5; //Emi will tell us probably 1.5

    constexpr static const OverTalonFXConfig IntakeConfig() {
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = 28;
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
        intakeJawConfig.MotorId = 29;
        intakeJawConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeJawConfig.Inverted = true;

        intakeJawConfig.ClosedLoopRampRate = 0.05_s;
        intakeJawConfig.CurrentLimit = 40_A;
        intakeJawConfig.StatorCurrentLimit = 120_A;
        intakeJawConfig.TriggerThreshold = 60_A;
        intakeJawConfig.TriggerThresholdTime = 1_s;
        intakeJawConfig.OpenLoopRampRate = 0.0_s;

        intakeJawConfig.PIDConfigs.WithKV(0.0).WithKP(0.0);

        return intakeJawConfig;
    }

};
