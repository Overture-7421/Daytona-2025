#pragma once

#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct ElevatorConstants {
    constexpr static const units::meter_t RangeError = 0.025_m;

    constexpr static const double LowerSensorToMechanism = 5.6;
    constexpr static const units::meter_t Diameter = 0.07366_m;

    constexpr static const units::volt_t feedForward = 0.37_V;

    constexpr static const units::turns_per_second_t ElevatorCruiseVelocity = 50.0_tps;
    constexpr static const units::turns_per_second_squared_t ElevatorCruiseAcceleration = 45_tr_per_s_sq;

    constexpr static const units::turns_per_second_t ElevatorLowerCruiseVelocity = 40.0_tps;
    constexpr static const units::turns_per_second_squared_t ElevatorLowerCruiseAcceleration = 20_tr_per_s_sq;

    constexpr static const units::meter_t CoralGroundGrabPosition = 0.08_m;
    constexpr static const units::meter_t AlgaeGroundGrabPosition = 0.05_m;
    constexpr static const units::meter_t CoralStationPosition = 0.06_m;
    constexpr static const units::meter_t ClosedPosition = 0.00_m;
    constexpr static const units::meter_t L1Position = 0.00_m;
    constexpr static const units::meter_t L2Position = 0.15_m;
    constexpr static const units::meter_t L3Position = 0.67_m;
    constexpr static const units::meter_t L4Position = 1.46_m;

    constexpr static const units::meter_t L2SpitPosition = 0.05_m;
    constexpr static const units::meter_t L3SpitPosition = 0.50_m;
    constexpr static const units::meter_t L4SpitPosition = 1.30_m;

    constexpr static const units::meter_t L4PositionAuto = 1.54_m;
    constexpr static const units::meter_t NetPosition = 1.65_m;
    constexpr static const units::meter_t ProcessorPosition = 0.00_m; //Falta
    constexpr static const units::meter_t LowAlgae = 0.32_m;
    constexpr static const units::meter_t HighAlgae = 0.85_m;
    constexpr static const units::meter_t HighMotionAllowed = 0.20_m; //Cambiar

    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig right;
        right.MotorId = 21;
        right.NeutralMode = ControllerNeutralMode::Brake;
        right.Inverted = true;
        right.useFOC = true;
        right.CurrentLimit = 20_A;
        right.StatorCurrentLimit = 120_A;
        right.TriggerThreshold = 40_A;
        right.TriggerThresholdTime = 0.5_s;
        right.ClosedLoopRampRate = 0.05_s;

        return right;
    }

    constexpr static const OverTalonFXConfig LeftConfig() {
        OverTalonFXConfig left;
        left.MotorId = 20;
        left.NeutralMode = ControllerNeutralMode::Brake;
        left.Inverted = false;
        left.useFOC = true;
        left.PIDConfigs.WithKV(1.7).WithKP(14.1);
        left.CurrentLimit = 20_A;
        left.StatorCurrentLimit = 120_A;
        left.TriggerThreshold = 40_A;
        left.TriggerThresholdTime = 0.5_s;
        left.ClosedLoopRampRate = 0.05_s;

        return left;

    }
};
