#pragma once

#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct ElevatorConstants {
    constexpr static const double LowerSensorToMechanism = 5.6;
    constexpr static const units::meter_t Diameter = 0.07366_m;

    constexpr static const units::turns_per_second_t ElevatorCruiseVelocity = 5.0_tps;
    constexpr static const units::turns_per_second_squared_t ElevatorCruiseAcceleration = 15_tr_per_s_sq;

    constexpr static const units::meter_t SourcePosition = 0.01_m;
    constexpr static const units::meter_t FloorPosition = 0_m;
    constexpr static const units::meter_t ClosedPosition = 0.02_m;
    constexpr static const units::meter_t L1Position = 0.2_m;
    constexpr static const units::meter_t L2Position = 0.03_m; 
    constexpr static const units::meter_t L3Position = 0.16_m;
    constexpr static const units::meter_t L4Position = 0.37_m;
    constexpr static const units::meter_t NetPosition = 0.36_m;
    constexpr static const units::meter_t ProcessorPosition = 0.05_m;
    constexpr static const units::meter_t LowAlgae = 0.15_m;
    constexpr static const units::meter_t HighAlgae = 0.25_m;
    constexpr static const units::meter_t HighMotionAllowed = 0.20_m;

    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig right;
        right.MotorId = 15;
        right.NeutralMode = ControllerNeutralMode::Brake;
        right.Inverted = true;
        right.useFOC = true;
        right.PIDConfigs.WithKP(8);
        right.CurrentLimit = 30_A;
        right.StatorCurrentLimit = 30_A;
        right.TriggerThreshold = 90_A;
        right.TriggerThresholdTime = 1_s;
        right.ClosedLoopRampRate = 0.05_s;
        right.OpenLoopRampRate = 0.05_s;

        return right;
    }

    constexpr static const OverTalonFXConfig LeftConfig() {
        OverTalonFXConfig left;
        left.MotorId = 14;
        left.NeutralMode = ControllerNeutralMode::Brake;
        left.Inverted = false;
        left.useFOC = true;
        left.PIDConfigs.WithKP(8);
        left.CurrentLimit = 30_A;
        left.StatorCurrentLimit = 30_A;
        left.TriggerThreshold = 90_A;
        left.TriggerThresholdTime = 1_s;
        left.ClosedLoopRampRate = 0.05_s;
        left.OpenLoopRampRate = 0.05_s;

        return left;

    }
};
