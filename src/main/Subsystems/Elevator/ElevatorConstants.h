#pragma once

#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct ElevatorConstants {
    constexpr static const units::meter_t RangeError = 0.02_m; // The math of the meter is wrong LOL

    constexpr static const double LowerSensorToMechanism = 5.6;
    constexpr static const units::meter_t Diameter = 0.07366_m;

    constexpr static const units::turns_per_second_t ElevatorCruiseVelocity = 40.0_tps; //5
    constexpr static const units::turns_per_second_squared_t ElevatorCruiseAcceleration = 35_tr_per_s_sq; //15

    constexpr static const units::meter_t CoralGroundGrabPosition = 0.08_m;
    constexpr static const units::meter_t AlgaeGroundGrabPosition = 0.08_m;
    constexpr static const units::meter_t CoralStationPosition = 0.01_m;
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
        right.MotorId = 21;
        right.NeutralMode = ControllerNeutralMode::Brake;
        right.Inverted = true;
        right.useFOC = true;
        right.CurrentLimit = 40_A;
        right.StatorCurrentLimit = 120_A;
        right.TriggerThreshold = 90_A;
        right.TriggerThresholdTime = 1_s;
        right.ClosedLoopRampRate = 0.05_s;

        return right;
    }

    constexpr static const OverTalonFXConfig LeftConfig() {
        OverTalonFXConfig left;
        left.MotorId = 20;
        left.NeutralMode = ControllerNeutralMode::Brake;
        left.Inverted = false;
        left.useFOC = true;
        left.PIDConfigs.WithKG(0.4).WithKV(1.5).WithKP(10.0);
        left.CurrentLimit = 40_A;
        left.StatorCurrentLimit = 120_A;
        left.TriggerThreshold = 90_A;
        left.TriggerThresholdTime = 1_s;
        left.ClosedLoopRampRate = 0.05_s;

        return left;

    }
};
