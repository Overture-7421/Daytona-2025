#pragma once
#include <units/length.h>
#include "Enums/Positions.h"
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>
#include <map>

struct ElevatorConstants {

    constexpr static const units::volt_t feedForward = 0_V;
    constexpr static const units::meter_t RangeError = 0.04_m; //Range of error the elevator is permiited to have.

    inline static const std::map<Positions, units::meter_t> ElevatorPositions = { {Positions::AlgaeGround, 0.10_m}, {
            Positions::AlgaeHold, 0.08_m}, {Positions::AlgaeHighReef, 1.30_m}, {Positions::AlgaeLowReef, 0.82_m}, {
            Positions::CoralAndAlgae, 1.05_m}, {Positions::CoralHold, 0.93_m}, {Positions::InitialPosition, 0.001_m}, {
            Positions::Intake, 1.05_m}, {Positions::IntakeCoralStation, 1.05_m}, {Positions::L1Confirm, 1.05_m}, {
            Positions::L1Position, 1.15_m}, {Positions::L2Back, 0.46_m}, {Positions::L2BackConfirm, 0.46_m}, {
            Positions::L2Front, 0.35_m}, {Positions::L2FrontConfirm, 0.35_m}, {Positions::L3Back, 0.87_m}, {
            Positions::L3BackConfirm, 0.87_m}, {Positions::L3Front, 0.95_m}, {Positions::L3FrontConfirm, 0.95_m}, {
            Positions::L4Back, 1.60_m}, {Positions::L4BackConfirm, 1.60_m}, {Positions::L4Front, 1.60_m}, {
            Positions::L4FrontConfirm, 1.60_m}, {Positions::NetPosition, 1.64_m}, {Positions::NetConfirm, 1.64_m}, {
            Positions::ProcessorPosition, 0.35_m}, {Positions::ProcessorConfirm, 0.35_m}, {Positions::SustainedPosition,
            1.05_m}, {Positions::EndPosition, 0.0_m}};

    constexpr static const units::turns_per_second_t ElevatorCruiseVelocity = 120.0_tps; //The velocity at which the elevator travels
    constexpr static const units::turns_per_second_squared_t ElevatorUpperCruiseAcceleration = 75_tr_per_s_sq; //The acceleration the elevator gains when going up
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
        right.CurrentLimit = 25_A;
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
        left.PIDConfigs.WithKG(0.18).WithKS(0.4).WithKP(30); //G=0.37   S=0.5  P=20
        left.CurrentLimit = 25_A;
        left.StatorCurrentLimit = 120_A;
        left.TriggerThreshold = 40_A;
        left.TriggerThresholdTime = 0.5_s;
        left.ClosedLoopRampRate = 0.05_s;

        return left;
    }
};
