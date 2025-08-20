// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include <map>
#include "Enums/Heading.h"
#include "Enums/Positions.h"

struct IntakeValues {

    units::volt_t rollers;
    units::volt_t centering;
    units::degree_t intake;

};

struct IntakeConstants {

    inline static const std::map<Positions, IntakeValues> IntakePositions = {

    /*The intake subsystem consists in three motors running at the same time, hence three
     variables will be needed, the first volt refers to the centering motor, the secon to the
     rollers and finally the degree type variable refers to the pivot*/

    {Positions::AlgaeGround, {0.0_V, 0.0_V, 0_deg}}, {Positions::AlgaeHold, {0.0_V, 0.0_V, 0_deg}}, {
            Positions::AlgaeHighReef, {0.0_V, 0.0_V, 0_deg}}, {Positions::AlgaeLowReef, {0.0_V, 0.0_V, 0_deg}}, {
            Positions::CoralAndAlgae, {0.0_V, 0.0_V, 30_deg}}, //To be defined
            {Positions::CoralHold, {0.0_V, 0.0_V, 0_deg}}, //To be defined
            {Positions::InitialPosition, {0.0_V, 0.0_V, 0_deg}}, {Positions::Intake, {3.0_V, 4.0_V, 80_deg}}, //To be defined
            {Positions::IntakeCoralStation, {3.0_V, 4.0_V, 10_deg}}, //To be defined
            {Positions::L1Confirm, {-2.0_V, 0.0_V, 30_deg}}, //To be defined
            {Positions::L1Position, {0.0_V, 0.0_V, 30_deg}}, //To be defined
            {Positions::L2Back, {0.0_V, 0.0_V, 0_deg}}, {Positions::L2BackConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::L2Front, {0.0_V, 0.0_V, 0_deg}}, {Positions::L2FrontConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::L3Back, {0.0_V, 0.0_V, 0_deg}}, {Positions::L3BackConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::L3Front, {0.0_V, 0.0_V, 0_deg}}, {Positions::L3FrontConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::L4Back, {0.0_V, 0.0_V, 0_deg}}, {Positions::L4BackConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::L4Front, {0.0_V, 0.0_V, 0_deg}}, {Positions::L4FrontConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::NetPosition, {0.0_V, 0.0_V, 0_deg}}, {Positions::NetConfirm, {0.0_V, 0.0_V, 0_deg}}, {
                    Positions::ProcessorPosition, {0.0_V, 0.0_V, 0_deg}}, {Positions::ProcessorConfirm, {0.0_V, 0.0_V,
                    0_deg}}, {Positions::SustainedPosition, {0.0_V, 0.0_V, 0_deg}}, {Positions::EndPosition, {0.0_V,
                    0.0_V, 0.0_deg}}};

    constexpr static const units::degree_t IntakeRangeError = 1_deg;

    constexpr static const units::turns_per_second_t IntakeCruiseVelocity = 0_tps;
    constexpr static const units::turns_per_second_squared_t IntakeCruiseAcceleration = 0_tr_per_s_sq;

    constexpr static const double IntakeRotorToSensor = 0;

    constexpr static const double IntakeMotorId = 26;
    constexpr static const double IntakeCANCoderId = 27;

    constexpr static const double RollersMotorId = 29;
    constexpr static const double CenteringMotorId = 31;

    constexpr static const OverTalonFXConfig IntakeConfig() { //Limites cuestionables
        OverTalonFXConfig intakeConfig;
        intakeConfig.MotorId = IntakeMotorId;
        intakeConfig.NeutralMode = ControllerNeutralMode::Brake;
        intakeConfig.useFOC = true;
        intakeConfig.Inverted = true;

        intakeConfig.ClosedLoopRampRate = 0.05_s;
        intakeConfig.CurrentLimit = 30_A;
        intakeConfig.StatorCurrentLimit = 120_A;
        intakeConfig.TriggerThreshold = 40_A;
        intakeConfig.TriggerThresholdTime = 0.5_s;
        intakeConfig.PIDConfigs.GravityType = 1;
        intakeConfig.PIDConfigs.WithKG(0.0).WithKV(0.0).WithKP(0.0);

        return intakeConfig;
    }

    constexpr static const CanCoderConfig IntakeCANConfig() {
        CanCoderConfig intakeCANConfig;
        intakeCANConfig.CanCoderId = IntakeCANCoderId;
        intakeCANConfig.Offset = 0.0_tr;

        return intakeCANConfig;
    }

    constexpr static const OverTalonFXConfig RollersConfig() { //Limites cuestionables
        OverTalonFXConfig rollersConfig;
        rollersConfig.MotorId = RollersMotorId;
        rollersConfig.NeutralMode = ControllerNeutralMode::Brake;
        rollersConfig.Inverted = true;

        rollersConfig.CurrentLimit = 25_A;
        rollersConfig.StatorCurrentLimit = 120_A;
        rollersConfig.TriggerThreshold = 40_A;
        rollersConfig.TriggerThresholdTime = 0.5_s;
        rollersConfig.ClosedLoopRampRate = 0.0_s;
        rollersConfig.OpenLoopRampRate = 0.05_s;

        return rollersConfig;
    }

    constexpr static const OverTalonFXConfig CenteringConfig() { //Limites cuestionables
        OverTalonFXConfig centeringConfig;
        centeringConfig.MotorId = CenteringMotorId;
        centeringConfig.NeutralMode = ControllerNeutralMode::Brake;
        centeringConfig.Inverted = true;

        centeringConfig.CurrentLimit = 25_A;
        centeringConfig.StatorCurrentLimit = 120_A;
        centeringConfig.TriggerThreshold = 40_A;
        centeringConfig.TriggerThresholdTime = 0.5_s;
        centeringConfig.ClosedLoopRampRate = 0.0_s;
        centeringConfig.OpenLoopRampRate = 0.05_s;

        return centeringConfig;
    }

};
