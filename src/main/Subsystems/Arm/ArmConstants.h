// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include <map>
#include "Enums/Positions.h"

struct ArmConstants {

    constexpr static const units::degree_t ArmRangeError = 2.0_deg;

    inline static const std::map<Positions, units::degree_t> ArmFront { {Positions::InitialPosition, -90_deg}, {
            Positions::SustainedPosition, 90.0_deg}, {Positions::Intake, 90.0_deg}, {Positions::IntakeCoralStation,
            90_deg}, {Positions::AlgaeHighReef, 0.0_deg}, {Positions::AlgaeLowReef, 0.0_deg}, {Positions::AlgaeGround,
            30.0_deg}, {Positions::L1Position, 90_deg}, {Positions::L1Confirm, 90.0_deg}, {Positions::CoralHold,
            90.0_deg}, {Positions::CoralAndAlgae, -90.0_deg}, {Positions::AlgaeHold, -90.0_deg}, {Positions::L2Front,
            -54.0_deg}, {Positions::L3Front, 307.0_deg}, {Positions::L4Front, 303.0_deg}, {Positions::L2FrontConfirm,
            -1.0_deg}, {Positions::L3FrontConfirm, 362.0_deg}, {Positions::L4FrontConfirm, 342.0_deg}, {
            Positions::NetPosition, -120.0_deg}, {Positions::NetConfirm, -80.0_deg}, {Positions::ProcessorPosition,
            0.0_deg}, {Positions::ProcessorConfirm, 0.0_deg}, {Positions::EndPosition, -68.0_deg}, {
            Positions::L4FrontAuto, -57.0_deg}, {Positions::L4FrontAutoConfirm, -18.0_deg}};

    inline static const std::map<Positions, units::degree_t> ArmBack { {Positions::L2Back, 234.0_deg}, {
            Positions::L3Back, -127.0_deg}, {Positions::L4Back, -123.0_deg}, {Positions::L2BackConfirm, 179.0_deg}, {
            Positions::L3BackConfirm, -182.0_deg}, {Positions::L4BackConfirm, -162.0_deg}};

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 9_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 6_tr_per_s_sq;

    constexpr static const double ArmRotorToSensor = 60;

    constexpr static const double ArmMotorId = 23;
    constexpr static const double ArmCANCoderId = 25;

    constexpr static const OverTalonFXConfig ArmConfig() { //LImites cuestionables
        OverTalonFXConfig armConfig;
        armConfig.MotorId = ArmMotorId;
        armConfig.NeutralMode = ControllerNeutralMode::Brake;
        armConfig.useFOC = true;
        armConfig.Inverted = true;

        armConfig.ClosedLoopRampRate = 0.05_s;
        armConfig.CurrentLimit = 30_A;
        armConfig.StatorCurrentLimit = 120_A;
        armConfig.TriggerThreshold = 40_A;
        armConfig.TriggerThresholdTime = 0.5_s;
        armConfig.PIDConfigs.GravityType = 1;
        armConfig.PIDConfigs.WithKG(0.32).WithKV(0).WithKP(170);

        return armConfig;
    }

    constexpr static const CanCoderConfig ArmCANConfig() {
        CanCoderConfig armCANConfig;
        armCANConfig.CanCoderId = ArmCANCoderId;
        armCANConfig.Offset = -0.046142578125_tr;
        armCANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;

        return armCANConfig;
    }

};
