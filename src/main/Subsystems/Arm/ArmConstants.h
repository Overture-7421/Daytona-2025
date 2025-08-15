// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"



struct ArmConstants {

    constexpr static const units::degree_t ArmRangeError = 0.0_deg;

    constexpr static const units::degree_t ArmScoreL1 = 0_deg;

    constexpr static const units::degree_t FrontArmScoreL2 = 0_deg;
    constexpr static const units::degree_t FrontArmScoreL3 = 0_deg;
    constexpr static const units::degree_t FrontArmScoreL4 = 0_deg;

    constexpr static const units::degree_t BackArmScoreL2 = 0_deg;
    constexpr static const units::degree_t BackArmScoreL3 = 0_deg;
    constexpr static const units::degree_t BackArmScoreL4 = 0_deg;

    constexpr static const units::degree_t AutonomousCoralArm = 0_deg;

    constexpr static const units::degree_t ArmSustainPosition = 0_deg;
    constexpr static const units::degree_t ArmClosedPosition = 0_deg;

    constexpr static const units::degree_t ArmAlgaePosition = 0_deg;
    constexpr static const units::degree_t ArmNetPosition = 0_deg;
    constexpr static const units::degree_t GroundAlgae = 0_deg;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 0_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 0_tr_per_s_sq;

    constexpr static const double ArmRotorToSensor = 0;

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
        armConfig.PIDConfigs.WithKG(0.0).WithKV(0.0).WithKP(0.0);

        return armConfig;
    }

    constexpr static const CanCoderConfig ArmCANConfig() {
        CanCoderConfig armCANConfig;
        armCANConfig.CanCoderId = ArmCANCoderId;
        armCANConfig.Offset = 0.0_tr;

        return armCANConfig;
    }

};
