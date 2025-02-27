// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Intake/Intake.h"

struct IntakeConstants {
public:

    constexpr static const units::degree_t RangeError = 2.5_deg;

    constexpr static const units::meter_t SensorCoralDistance = 0.08_m;
    constexpr static const units::meter_t SensorAlgaeDistance = 0.08_m;

    constexpr static const units::degree_t JawAlgae = 73_deg;
    constexpr static const units::degree_t JawCoralOpen = 43_deg; //20
    constexpr static const units::degree_t JawCoralSpitL1 = 20_deg; //20
    constexpr static const units::degree_t JawCoralSpitL4 = 20_deg; //20
    constexpr static const units::degree_t JawCoralStation = 24_deg; //30
    constexpr static const units::degree_t JawCoralClose = 23_deg;

    constexpr static const units::volt_t CoralGrab = 2.5_V; //Positivo es para adentro
    constexpr static const units::volt_t CoralRelease = -2.5_V;
    constexpr static const units::volt_t CoralSpit = -1.8_V;
    constexpr static const units::volt_t CoralSpitL4 = -1.5_V;

    constexpr static const units::volt_t AlgaeGrab = 4.0_V;
    constexpr static const units::volt_t AlgaeRelease = -4.0_V;
    constexpr static const units::volt_t AlgaeHold = 6_V;

    constexpr static const units::volt_t SlowIntake = 3.0_V;
    constexpr static const units::volt_t StopIntake = 0.0_V;
    constexpr static const units::volt_t ReverseVolts = -4.0_V;

    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 40_tps;
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 30_tr_per_s_sq;

    constexpr static const double SensorToMechanism = 9.9519621;

    constexpr static const OverTalonFXConfig IntakeConfig() {
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = 28;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeConfig.Inverted = false;

        intakeConfig.CurrentLimit = 25_A;
        intakeConfig.StatorCurrentLimit = 120_A;
        intakeConfig.TriggerThreshold = 35_A;
        intakeConfig.TriggerThresholdTime = 0.5_s;
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
        intakeJawConfig.CurrentLimit = 30_A;
        intakeJawConfig.StatorCurrentLimit = 120_A;
        intakeJawConfig.TriggerThreshold = 40_A;
        intakeJawConfig.TriggerThresholdTime = 0.5_s;
        intakeJawConfig.OpenLoopRampRate = 0.0_s;

        intakeJawConfig.PIDConfigs.WithKV(0.8).WithKP(95.0);

        return intakeJawConfig;
    }

};
