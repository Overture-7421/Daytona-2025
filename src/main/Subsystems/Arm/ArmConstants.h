// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ArmConstants {
    constexpr static const units::degree_t ArmAngleRange = 1.2_deg;
    constexpr static const units::degree_t WristAngleRange = 1.7_deg; //Bajar tolerancia

    constexpr static const units::degree_t ArmScore = 40_deg;

    constexpr static const units::degree_t ArmCoralInter = 75_deg; //15
    constexpr static const units::degree_t ArmAlgaeInter = 60_deg;

    constexpr static const units::degree_t ArmL1Reef = 141.0_deg; //58
    constexpr static const units::degree_t WristL1Reef = 90.0_deg; //90

    constexpr static const units::degree_t ArmL2Reef = 58.0_deg;
    constexpr static const units::degree_t WristL2Reef = 0.0_deg;

    constexpr static const units::degree_t ArmL3Reef = 58.0_deg; //50
    constexpr static const units::degree_t WristL3Reef = 0.0_deg;

    constexpr static const units::degree_t ArmL4Reef = 53.0_deg; //65
    constexpr static const units::degree_t WristL4Reef = 0.0_deg;

    constexpr static const units::degree_t ArmL4ReefAuto = 57.0_deg; //65
    constexpr static const units::degree_t WristL4ReefAuto = 0.0_deg;

    constexpr static const units::degree_t ArmHighAlgae = 90.0_deg; //2
    constexpr static const units::degree_t WristHighAlgae = 0.0_deg;

    constexpr static const units::degree_t ArmLowAlgae = 90.0_deg; //2
    constexpr static const units::degree_t WristLowAlgae = 0.0_deg;

    constexpr static const units::degree_t ArmCoralStation = 121_deg; //95
    constexpr static const units::degree_t WristCoralStation = 90.0_deg;

    constexpr static const units::degree_t ArmProcessor = 0.0_deg; //90
    constexpr static const units::degree_t WristProcessor = 90.0_deg;

    constexpr static const units::degree_t ArmNet = 90.0_deg; // 1
    constexpr static const units::degree_t WristNet = 0.0_deg;

    constexpr static const units::degree_t ArmCoralGround = 0.0_deg; //120
    constexpr static const units::degree_t WristCoralGround = 90.0_deg;

    constexpr static const units::degree_t ArmAlgaeGround = 0.0_deg; //110
    constexpr static const units::degree_t WristAlgaeGround = 0.0_deg;

    constexpr static const units::degree_t ArmClosed = 90.0_deg; // 0
    constexpr static const units::degree_t WristClosed = 0.0_deg;

    constexpr static const double ArmRotorToSensor = 80.88889;
    constexpr static const double WristRotorToSensor = 31.5;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 6_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 2_tr_per_s_sq;
    constexpr static const units::turns_per_second_t WristCruiseVelocity = 20.0_tps;
    constexpr static const units::turns_per_second_squared_t WristCruiseAcceleration = 10_tr_per_s_sq;

    constexpr static const units::turns_per_second_t SlowVelocity = 3_tps;
    constexpr static const units::turns_per_second_squared_t SlowAccelerationa = 1_tr_per_s_sq;

    constexpr static const double ArmCANCoderId = 25;
    constexpr static const double WristCANCoderId = 27;

    constexpr static const double ArmLeftMotorId = 23;

    constexpr static const double WristMotorId = 26;

    constexpr static const OverTalonFXConfig ArmLeftConfig() {
        OverTalonFXConfig armLeftConfig;
        armLeftConfig.MotorId = ArmLeftMotorId;
        armLeftConfig.NeutralMode = ControllerNeutralMode::Brake;
        armLeftConfig.useFOC = true;
        armLeftConfig.Inverted = true;

        armLeftConfig.ClosedLoopRampRate = 0.05_s;
        armLeftConfig.CurrentLimit = 20_A;
        armLeftConfig.StatorCurrentLimit = 120_A;
        armLeftConfig.TriggerThreshold = 30_A;
        armLeftConfig.TriggerThresholdTime = 0.5_s;
        armLeftConfig.PIDConfigs.GravityType = 1;
        armLeftConfig.PIDConfigs.WithKV(1.8).WithKP(180);

        return armLeftConfig;
    }

    constexpr static const OverTalonFXConfig WristConfig() {
        OverTalonFXConfig wristConfig;
        wristConfig.MotorId = WristMotorId;
        wristConfig.NeutralMode = ControllerNeutralMode::Brake;
        wristConfig.useFOC = true;

        wristConfig.ClosedLoopRampRate = 0.05_s;
        wristConfig.CurrentLimit = 25_A;
        wristConfig.StatorCurrentLimit = 120_A;
        wristConfig.TriggerThreshold = 30_A;
        wristConfig.TriggerThresholdTime = 0.5_s;
        wristConfig.PIDConfigs.WithKV(1.8).WithKP(70);

        return wristConfig;
    }

    constexpr static const CanCoderConfig ArmCANConfig() {
        CanCoderConfig armCANConfig;
        armCANConfig.CanCoderId = ArmCANCoderId;
        armCANConfig.Offset = -0.041259765625_tr;

        return armCANConfig;
    }

    constexpr static const CanCoderConfig WristCANConfig() {
        CanCoderConfig wristCANConfig;
        wristCANConfig.CanCoderId = WristCANCoderId;
        wristCANConfig.Offset = 0.1787109375_tr;
        wristCANConfig.SensorDirection = 1;

        return wristCANConfig;
    }

};
