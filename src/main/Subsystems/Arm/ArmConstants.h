// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ArmConstants {
    constexpr static const units::degree_t ArmAngleRange = 1.5_deg;
    constexpr static const units::degree_t WristAngleRange = 1.5_deg;

    constexpr static const units::degree_t ArmL1Reef = 100.0_deg;
    constexpr static const units::degree_t WristL1Reef = 90.0_deg;

    constexpr static const units::degree_t ArmL2Reef = 35.0_deg;
    constexpr static const units::degree_t WristL2Reef = 0.0_deg;

    constexpr static const units::degree_t ArmL3Reef = 30.0_deg;
    constexpr static const units::degree_t WristL3Reef = 1.0_deg;

    constexpr static const units::degree_t ArmL4Reef = 25.0_deg;
    constexpr static const units::degree_t WristL4Reef = 1.0_deg;

    constexpr static const units::degree_t ArmHighAlgae = 68.0_deg;
    constexpr static const units::degree_t WristHighAlgae = 90.0_deg;

    constexpr static const units::degree_t ArmLowAlgae = 70.0_deg;
    constexpr static const units::degree_t WristLowhAlgae = 90.0_deg;

    constexpr static const units::degree_t ArmCoralStation = -31_deg;
    constexpr static const units::degree_t WristCoralStation = 90.0_deg;

    constexpr static const units::degree_t ArmProcessor = 90.0_deg;
    constexpr static const units::degree_t WristProcessor = 90.0_deg;

    constexpr static const units::degree_t ArmNet = 1.0_deg;
    constexpr static const units::degree_t WristNet = 1.0_deg;

    constexpr static const units::degree_t ArmGround = 120.0_deg;
    constexpr static const units::degree_t WristGround = 90.0_deg;

    constexpr static const units::degree_t ArmClosed = 0.0_deg;
    constexpr static const units::degree_t WristClosed = 0.0_deg;

    constexpr static const double ArmRotorToSensor = 80.88888;
    constexpr static const double WristRotorToSensor = 27;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 2.5_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 7_tr_per_s_sq;
    constexpr static const units::turns_per_second_t WristCruiseVelocity = 10.0_tps;
    constexpr static const units::turns_per_second_squared_t WristCruiseAcceleration = 18_tr_per_s_sq;

    constexpr static const double ArmCANCoderId = 17;
    constexpr static const double WristCANCoderId = 19;

    constexpr static const double ArmLeftMotorId = 15;
    constexpr static const double ArmRightMotorId = 16;

    constexpr static const double WristMotorId = 18;

    

    constexpr static const OverTalonFXConfig ArmLeftConfig() {
        OverTalonFXConfig armLeftConfig;
        armLeftConfig.MotorId = ArmLeftMotorId;
        armLeftConfig.NeutralMode = ControllerNeutralMode::Brake;
        armLeftConfig.useFOC = true;

        armLeftConfig.ClosedLoopRampRate = 0.05_s;
        armLeftConfig.CurrentLimit = 30_A;
        armLeftConfig.StatorCurrentLimit = 30_A;
        armLeftConfig.TriggerThreshold = 90_A;
        armLeftConfig.TriggerThresholdTime = 1_s;
        armLeftConfig.OpenLoopRampRate = 0.05_s;
        armLeftConfig.PIDConfigs.WithKP(22.0);

        return armLeftConfig;
    }

    constexpr static const OverTalonFXConfig ArmRightConfig() {
        OverTalonFXConfig armRightConfig;
        armRightConfig.MotorId = ArmRightMotorId;
        armRightConfig.NeutralMode = ControllerNeutralMode::Brake;
        armRightConfig.useFOC = true;

        armRightConfig.ClosedLoopRampRate = 0.05_s;
        armRightConfig.CurrentLimit = 30_A;
        armRightConfig.StatorCurrentLimit = 30_A;
        armRightConfig.TriggerThreshold = 90_A;
        armRightConfig.TriggerThresholdTime = 1_s;
        armRightConfig.OpenLoopRampRate = 0.05_s;
        armRightConfig.PIDConfigs.WithKP(22.0);

        return armRightConfig;
    }

    constexpr static const OverTalonFXConfig WristConfig() {
        OverTalonFXConfig wristConfig;
        wristConfig.MotorId = WristMotorId;
        wristConfig.NeutralMode = ControllerNeutralMode::Brake;
        wristConfig.useFOC = true;

        wristConfig.ClosedLoopRampRate = 0.05_s;
        wristConfig.CurrentLimit = 30_A;
        wristConfig.StatorCurrentLimit = 30_A;
        wristConfig.TriggerThreshold = 90_A;
        wristConfig.TriggerThresholdTime = 1_s;
        wristConfig.OpenLoopRampRate = 0.05_s;
        wristConfig.PIDConfigs.WithKP(8.0);

        return wristConfig;
    }

    constexpr static const CanCoderConfig ArmCANConfig() {
        CanCoderConfig armCANConfig;
        armCANConfig.CanCoderId = ArmCANCoderId;
        armCANConfig.Offset = 0.0_tr;

        return armCANConfig;
    }

    constexpr static const CanCoderConfig WristCANConfig() {
        CanCoderConfig wristCANConfig;
        wristCANConfig.CanCoderId = WristCANCoderId;
        wristCANConfig.Offset = 0.0_tr;

        return wristCANConfig;
    }

};
