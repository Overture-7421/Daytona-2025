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

    constexpr static const units::meter_t SourcePosition = 2.13_m;
    constexpr static const units::meter_t FloorPosition = 0_m;
    constexpr static const units::meter_t ClosedPosition = 2.13_m;
    constexpr static const units::meter_t L1Position = 0.46_m;
    constexpr static const units::meter_t L2Position = 0.81_m;
    constexpr static const units::meter_t L3Position = 1.21_m;
    constexpr static const units::meter_t L4Position = 1.83_m;
    constexpr static const units::meter_t NetPosition = 2.5_m;
    constexpr static const units::meter_t ProcessorPosition = 0.1_m;
    constexpr static const units::meter_t BottomAlgaePosition = 0.95_m;
    constexpr static const units::meter_t TopAlgaePosition = 1.8_m;

    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig right;
        right.MotorId = 15;
        right.NeutralMode = ControllerNeutralMode::Brake;
        right.Inverted = true;
        right.useFOC = true;
        right.PIDConfigs.WithKP(17);
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
        left.PIDConfigs.WithKP(17);
        left.CurrentLimit = 30_A;
        left.StatorCurrentLimit = 30_A;
        left.TriggerThreshold = 90_A;
        left.TriggerThresholdTime = 1_s;
        left.ClosedLoopRampRate = 0.05_s;
        left.OpenLoopRampRate = 0.05_s;

        return left;

    }
};
