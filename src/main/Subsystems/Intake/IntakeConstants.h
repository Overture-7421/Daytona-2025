// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include <map>
#include "Enums/Heading.h"
#include "Enums/Positions.h"

struct IntakeConstants {

    inline static const std::map<Positions, units::volt_t, units::degree_t > IntakePositions = {
        {Positions::AlgaeGround, 0.0_V, }, 
        {Positions::AlgaeHold, 2.0_v},
        {Positions::AlgaeHighReef,8.0_v},
        {Positions::AlgaeLowReef, 8.0_v},
        {Positions::CoralandAlgae, 8.0_v},
        {Positions::CoralHold, 0.0_V},
        {Positions::InitialPosition, 0.0_V},
        {Positions::Intake, 4.0_V},
        {Positions::IntakeCoralStation, 0.0_V},
        {Positions::L1Confirm, 0.0_V},
        {Positions::L1Position, 0.0_V},
        {Positions::L2Back, 0.0_V},
        {Positions::L2BackConfirm, 0.0_V},
        {Positions::L2Front, 0.0_V},
        {Positions::L2FrontConfirm, 0.0_V},
        {Positions::L3Back, 0.0_V},
        {Positions::L3BackConfirm, 0.0_V},
        {Positions::L3Front, 0.0_V},
        {Positions::L3FrontConfirm, 0.0_V},
        {Positions::L4Back, 0.0_V},
        {Positions::L4BackConfirm, 0.0_V},
        {Positions::L4Front, 0.0_V},
        {Positions::L4FrontConfirm, 0.0_V},
        {Positions::NetPosition,2.0_V},
        {Positions::NetConfirm, -4.0_V},
        {Positions::ProcessorPosition, 2.0_V},
        {Positions::ProcessorConfirm, -4.0_V},
        {Positions::SustainedPosition, 0.0_V}        
    }

    constexpr static const units::degree_t IntakeRangeError = 1_deg;

    constexpr static const units::degree_t L1Intake = 0_deg;
    constexpr static const units::volt_t L1Rollers = 0_V;
    constexpr static const units::volt_t L1Centering = 0_V;

  
    constexpr static const units::degree_t CoralGroundIntake = 0_deg;
    constexpr static const units::volt_t CoralGroundRollers = 0_V;
    constexpr static const units::volt_t CoralGroundCentering = 0_V;

    constexpr static const units::degree_t ClosedIntake = 0_deg;
    constexpr static const units::volt_t closedRollers = 0_V;
    constexpr static const units::volt_t ClosedCentering = 0_V;

    constexpr static const units::degree_t CoralStationIntake = 0_deg;
    constexpr static const units::volt_t CoralStationRollers = 0_V;
    constexpr static const units::volt_t CoralStationCentering = 0_V;

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
