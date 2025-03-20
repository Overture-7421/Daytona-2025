// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Intake/Intake.h"

struct IntakeConstants {
public:

    constexpr static const units::meter_t SensorCoralDistance = 0.08_m;
    constexpr static const units::meter_t SensorAlgaeDistance = 0.08_m;

    constexpr static const units::volt_t CoralGrab = 4.0_V; //Positivo es para adentro
    constexpr static const units::volt_t CoralGroundGrab = 6.3_V;
    constexpr static const units::volt_t CoralRelease = -2.5_V;
    constexpr static const units::volt_t CoralSpit = -2.5_V;
    constexpr static const units::volt_t CoralSpitL4 = -1.5_V;

    constexpr static const units::volt_t AlgaeGrab = -5.0_V; //7
    constexpr static const units::volt_t AlgaeRelease = 8.0_V;

    constexpr static const units::volt_t AlgaeHold = -0.8_V; //-0.8

    constexpr static const units::volt_t SlowIntake = 3.0_V;
    constexpr static const units::volt_t StopIntake = 0.0_V;
    constexpr static const units::volt_t ReverseVolts = -4.0_V;

    constexpr static const OverTalonFXConfig IntakeConfig() {
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = 28;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeConfig.Inverted = true;

        intakeConfig.CurrentLimit = 25_A;
        intakeConfig.StatorCurrentLimit = 120_A;
        intakeConfig.TriggerThreshold = 40_A;
        intakeConfig.TriggerThresholdTime = 0.5_s;
        intakeConfig.ClosedLoopRampRate = 0.0_s;
        intakeConfig.OpenLoopRampRate = 0.05_s;

        return intakeConfig;
    }

};
