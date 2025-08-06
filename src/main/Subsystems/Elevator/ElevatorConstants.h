#pragma once
#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct ElevatorConstants {

    constexpr static const units::volt_t feedForward = 0_V;
    constexpr static const units::meter_t RangeError = 0.025_m; //Range of error the elevator is permiited to have.

    constexpr static const units::meter_t ClosedPosition = 0.00_m;

    //Positions for each level at the reef in which the arm can "hover"
    constexpr static const units::meter_t L2Position = 0.23_m;
    constexpr static const units::meter_t L3Position = 0.76_m;
    constexpr static const units::meter_t L4Position = 1.62_m;

    //Positions for each level at the reef in which the arm can "score"
    constexpr static const units::meter_t L2ScorePosition = 0.00_m;
    constexpr static const units::meter_t L3ScorePosition = 0.40_m;
    constexpr static const units::meter_t L4ScorePosition = 1.00_m;

    //All positions that involve an algae
    constexpr static const units::meter_t NetPosition = 1.67_m;
    constexpr static const units::meter_t ProcessorPosition = 0.48_m;
    constexpr static const units::meter_t LowAlgaePosition = 0.46_m;
    constexpr static const units::meter_t HighAlgaePosition = 1.00_m;
    constexpr static const units::meter_t GroundAlgaePosition = 0.48_m;

    constexpr static const units::turns_per_second_t ElevatorCruiseVelocity = 100.0_tps; //The velocity at which the elevator travels
    constexpr static const units::turns_per_second_squared_t ElevatorUpperCruiseAcceleration = 65_tr_per_s_sq; //The acceleration the elevator gains when going up
    constexpr static const units::turns_per_second_squared_t ElevatorLowerCruiseAcceleration = 20_tr_per_s_sq; //The acceleration the elevator gains when going down

    constexpr static const double LowerSensorToMechanism = 5.6; //The gear ratio there exists between the encoder to the actual mechanism.
    constexpr static const units::meter_t Diameter = 0.07366_m; //Diameter of the "pulley" neeeded for the elevator

    //Configuration of the motors the elevator uses
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
        left.PIDConfigs.WithKG(0.37).WithKS(0.5).WithKP(20); //KV1.7 P 14.1
        left.CurrentLimit = 25_A;
        left.StatorCurrentLimit = 120_A;
        left.TriggerThreshold = 40_A;
        left.TriggerThresholdTime = 0.5_s;
        left.ClosedLoopRampRate = 0.05_s;

        return left;
    }
};
