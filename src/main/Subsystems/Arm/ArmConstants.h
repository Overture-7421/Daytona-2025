// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ArmConstants {
    constexpr static const units::degree_t ArmAngleRange = 1.5_deg;
    constexpr static const units::degree_t WristAngleRange = 1.5_deg;

    constexpr static const units::degree_t ArmCoralInter = 75_deg; //15
    constexpr static const units::degree_t ArmAlgaeInter = 60_deg;

    constexpr static const units::degree_t ArmL1Reef = 0.0_deg; // Antes era 100_deg
    constexpr static const units::degree_t WristL1Reef = 90.0_deg;

    constexpr static const units::degree_t ArmL2Reef = 55.0_deg; // 35
    constexpr static const units::degree_t WristL2Reef = 0.0_deg;

    constexpr static const units::degree_t ArmL3Reef = 60.0_deg; //30
    constexpr static const units::degree_t WristL3Reef = 0.0_deg;

    constexpr static const units::degree_t ArmL4Reef = 65.0_deg; //25
    constexpr static const units::degree_t WristL4Reef = 0.0_deg;

    constexpr static const units::degree_t ArmHighAlgae = 22.0_deg; //68
    constexpr static const units::degree_t WristHighAlgae = 90.0_deg;

    constexpr static const units::degree_t ArmLowAlgae = 20.0_deg; //70
    constexpr static const units::degree_t WristLowAlgae = 90.0_deg;

    constexpr static const units::degree_t ArmCoralStation = 121_deg; //-31
    constexpr static const units::degree_t WristCoralStation = 90.0_deg;

    constexpr static const units::degree_t ArmProcessor = 0.0_deg; //90
    constexpr static const units::degree_t WristProcessor = 90.0_deg;

    constexpr static const units::degree_t ArmNet = 89.0_deg; // 1
    constexpr static const units::degree_t WristNet = 1.0_deg;

    constexpr static const units::degree_t ArmCoralGround = 0.0_deg; //120
    constexpr static const units::degree_t WristCoralGround = 90.0_deg;

    constexpr static const units::degree_t ArmAlgaeGround = 0.0_deg; //110
    constexpr static const units::degree_t WristAlgaeGround = 0.0_deg;

    constexpr static const units::degree_t ArmClosed = 90.0_deg; // 0
    constexpr static const units::degree_t WristClosed = 0.0_deg;

    constexpr static const double ArmRotorToSensor = 80.88889;
    constexpr static const double WristRotorToSensor = 31.5;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 100_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 90_tr_per_s_sq;
    constexpr static const units::turns_per_second_t WristCruiseVelocity = 90.0_tps;
    constexpr static const units::turns_per_second_squared_t WristCruiseAcceleration = 80_tr_per_s_sq;

    constexpr static const double ArmCANCoderId = 25;
    constexpr static const double WristCANCoderId = 27;

    constexpr static const double ArmLeftMotorId = 23;
    constexpr static const double ArmRightMotorId = 24; //Borrar

    constexpr static const double WristMotorId = 26;

    constexpr static const OverTalonFXConfig ArmLeftConfig() {
        OverTalonFXConfig armLeftConfig;
        armLeftConfig.MotorId = ArmLeftMotorId;
        armLeftConfig.NeutralMode = ControllerNeutralMode::Brake;
        armLeftConfig.useFOC = true;
        armLeftConfig.Inverted = true;

        armLeftConfig.ClosedLoopRampRate = 0.05_s;
        armLeftConfig.CurrentLimit = 30_A;
        armLeftConfig.StatorCurrentLimit = 120_A;
        armLeftConfig.TriggerThreshold = 90_A;
        armLeftConfig.TriggerThresholdTime = 1_s;
        armLeftConfig.PIDConfigs.GravityType = 1;
        armLeftConfig.PIDConfigs.WithKG(0.14).WithKV(2.0).WithKP(75.0);

        return armLeftConfig;
    }

    constexpr static const OverTalonFXConfig WristConfig() {
        OverTalonFXConfig wristConfig;
        wristConfig.MotorId = WristMotorId;
        wristConfig.NeutralMode = ControllerNeutralMode::Brake;
        wristConfig.useFOC = true;

        wristConfig.ClosedLoopRampRate = 0.05_s;
        wristConfig.CurrentLimit = 30_A;
        wristConfig.StatorCurrentLimit = 120_A;
        wristConfig.TriggerThreshold = 90_A;
        wristConfig.TriggerThresholdTime = 1_s;
        wristConfig.PIDConfigs.WithKV(1.5).WithKP(60.0);

        return wristConfig;
    }

    constexpr static const CanCoderConfig ArmCANConfig() {
        CanCoderConfig armCANConfig;
        armCANConfig.CanCoderId = ArmCANCoderId;
        armCANConfig.Offset = -0.01953125_tr;

        return armCANConfig;
    }

    constexpr static const CanCoderConfig WristCANConfig() {
        CanCoderConfig wristCANConfig;
        wristCANConfig.CanCoderId = WristCANCoderId;
        wristCANConfig.Offset = 0.345703125_tr;
        wristCANConfig.SensorDirection = 1;

        return wristCANConfig;
    }

};
