#pragma once

#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct ElevatorConstants {
    constexpr static const units::meter_t RangeError = 0.03_m;

    constexpr static const double LowerSensorToMechanism = 5.6;
    constexpr static const units::meter_t Diameter = 0.07366_m;

    constexpr static const units::turns_per_second_t ElevatorCruiseVelocity = 40.0_tps;
    constexpr static const units::turns_per_second_squared_t ElevatorCruiseAcceleration = 20_tr_per_s_sq; //Bajar Acceleracion 30 es mucho

    constexpr static const units::meter_t CoralGroundGrabPosition = 0.08_m; //Falta
    constexpr static const units::meter_t AlgaeGroundGrabPosition = 0.08_m; //falta
    constexpr static const units::meter_t CoralStationPosition = 0.01_m;
    constexpr static const units::meter_t ClosedPosition = 0.04_m;
    constexpr static const units::meter_t L1Position = 0.35_m;
    constexpr static const units::meter_t L2Position = 0.15_m;
    constexpr static const units::meter_t L3Position = 0.70_m;
    constexpr static const units::meter_t L4Position = 1.53_m;
    constexpr static const units::meter_t NetPosition = 1.65_m;
    constexpr static const units::meter_t ProcessorPosition = 0.05_m; //Falta
    constexpr static const units::meter_t LowAlgae = 0.75_m;
    constexpr static const units::meter_t HighAlgae = 1.20_m;
    constexpr static const units::meter_t HighMotionAllowed = 0.20_m; //Cambiar

    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig right;
        right.MotorId = 21;
        right.NeutralMode = ControllerNeutralMode::Coast;
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
        left.PIDConfigs.WithKG(0.5).WithKV(1.5).WithKP(12.0);
        left.CurrentLimit = 40_A;
        left.StatorCurrentLimit = 120_A;
        left.TriggerThreshold = 90_A;
        left.TriggerThresholdTime = 1_s;
        left.ClosedLoopRampRate = 0.05_s;

        return left;

    }
};
