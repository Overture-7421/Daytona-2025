// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ArmConstants {

    constexpr static const double ArmMotorId = 23;
    constexpr static const double ArmCANCoderId = 25;

    constexpr static const OverTalonFXConfig ArmLeftConfig() {
        OverTalonFXConfig armLeftConfig;
        armLeftConfig.MotorId = ArmMotorId;
        armLeftConfig.NeutralMode = ControllerNeutralMode::Brake;
        armLeftConfig.useFOC = true;
        armLeftConfig.Inverted = true;

        armLeftConfig.ClosedLoopRampRate = 0.05_s;
        armLeftConfig.CurrentLimit = 30_A;
        armLeftConfig.StatorCurrentLimit = 120_A;
        armLeftConfig.TriggerThreshold = 40_A;
        armLeftConfig.TriggerThresholdTime = 0.5_s;
        armLeftConfig.PIDConfigs.GravityType = 1;
        armLeftConfig.PIDConfigs.WithKG(0.0).WithKV(0.0).WithKP(0.0);

        return armLeftConfig;
    }

    constexpr static const CanCoderConfig ArmCANConfig() {
        CanCoderConfig armCANConfig;
        armCANConfig.CanCoderId = ArmCANCoderId;
        armCANConfig.Offset = 0.0_tr;

        return armCANConfig;
    }

};
