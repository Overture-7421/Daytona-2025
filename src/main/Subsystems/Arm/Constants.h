// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct Constants {
    constexpr static const units::degree_t ArmAngleRange = 1.0_deg;
    constexpr static const units::degree_t WristAngleRange = 1.0_deg;

    constexpr static const units::degree_t ArmL1Reef = 1.0_deg;
    constexpr static const units::degree_t WristL1Reef = 1.0_deg;

    constexpr static const units::degree_t ArmL2Reef = 1.0_deg;
    constexpr static const units::degree_t WristL2Reef = 1.0_deg;

    constexpr static const units::degree_t ArmL3Reef = 1.0_deg;
    constexpr static const units::degree_t WristL3Reef = 1.0_deg;

    constexpr static const units::degree_t ArmL4Reef = 1.0_deg;
    constexpr static const units::degree_t WristL4Reef = 1.0_deg;

    constexpr static const units::degree_t ArmCoralStation = 1.0_deg;
    constexpr static const units::degree_t WristCoralStation = 1.0_deg;

    constexpr static const units::degree_t ArmProcessor = 1.0_deg;
    constexpr static const units::degree_t WristProcessor = 1.0_deg;

    constexpr static const units::degree_t ArmNet = 1.0_deg;
    constexpr static const units::degree_t WristNet = 1.0_deg;

    constexpr static const units::degree_t ArmGround = 1.0_deg;
    constexpr static const units::degree_t WristGround = 1.0_deg;

    constexpr static const units::degree_t ArmClosed = 1.0_deg;
    constexpr static const units::degree_t WristClosed = 1.0_deg;

    constexpr static const double ArmRotorToSensor = 1.0;
    constexpr static const double WristRotorToSensor = 1.0;
    constexpr static const double ArmSensorToMechanism = 1.0;
    constexpr static const double WristSensorToMechanism = 1.0;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 1.5_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 7_tr_per_s_sq;
    constexpr static const units::turns_per_second_t WristCruiseVelocity = 5.0_tps;
    constexpr static const units::turns_per_second_squared_t WristCruiseAcceleration = 15_tr_per_s_sq;

    constexpr static const double ArmCANCoderId = 22;
    constexpr static const double WristCANCoderId = 23;

    constexpr static const double ArmLeftMotorId = 20;
    constexpr static const double ArmRightMotorId = 29;

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
        armLeftConfig.PIDConfigs.WithKP(0.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

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
        armRightConfig.PIDConfigs.WithKP(0.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

        return armRightConfig;
    }

    constexpr static const OverTalonFXConfig WristConfig() {
        OverTalonFXConfig wristConfig;
        wristConfig.MotorId = 21;
        wristConfig.NeutralMode = ControllerNeutralMode::Brake;
        wristConfig.useFOC = true;

        wristConfig.ClosedLoopRampRate = 0.05_s;
        wristConfig.CurrentLimit = 30_A;
        wristConfig.StatorCurrentLimit = 30_A;
        wristConfig.TriggerThreshold = 90_A;
        wristConfig.TriggerThresholdTime = 1_s;
        wristConfig.OpenLoopRampRate = 0.05_s;
        wristConfig.PIDConfigs.WithKP(0.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

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
