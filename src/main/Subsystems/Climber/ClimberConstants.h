#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

#include "Enums/Positions.h"

struct ClimberConstants {

    constexpr static const double ClimberRangeError = 0.0;

    inline static const std::map<Positions, double> ClimberPositions = { {Positions::AlgaeGround, 0.0}, {
            Positions::AlgaeHold, 0.0}, {Positions::AlgaeHighReef, 0.0}, {Positions::AlgaeLowReef, 0.0}, {
            Positions::CoralAndAlgae, 0.0}, {Positions::CoralHold, 0.0}, {Positions::InitialPosition, 0.0}, {
            Positions::Intake, 0.0}, {Positions::IntakeCoralStation, 0.0}, {Positions::L1Confirm, 0.0}, {
            Positions::L1Position, 0.0}, {Positions::L2Back, 0.0}, {Positions::L2BackConfirm, 0.0}, {Positions::L2Front,
            0.0}, {Positions::L2FrontConfirm, 0.0}, {Positions::L3Back, 0.0}, {Positions::L3BackConfirm, 0.0}, {
            Positions::L3Front, 0.0}, {Positions::L3FrontConfirm, 0.0}, {Positions::L4Back, 0.0}, {
            Positions::L4BackConfirm, 0.0}, {Positions::L4Front, 0.0}, {Positions::L4FrontConfirm, 0.0}, {
            Positions::NetPosition, 0.0}, {Positions::NetConfirm, 0.0}, {Positions::ProcessorPosition, 0.0}, {
            Positions::ProcessorConfirm, 0.0}, {Positions::SustainedPosition, 0.0}, {Positions::EndPosition, 0.0}};

    constexpr static const units::turns_per_second_t ClimberCruiseVelocity = 0.0_ps;
    constexpr static const units::turns_per_second_squared_t ClimberCruiseAcceleration = 0.0_mps_sq;

    constexpr static const double ClimberEncoderOffset = 0;
    constexpr static const double ClimberSensorToMechanism = 0;

    constexpr static const double ClimberMotorId = 22;

    constexpr static const OverTalonFXConfig ClimberConfig() {
        OverTalonFXConfig climberConfig;
        climberConfig.MotorId = ClimberMotorId;
        climberConfig.NeutralMode = ControllerNeutralMode::Brake;
        climberConfig.Inverted = true;
        climberConfig.useFOC = true;

        climberConfig.CurrentLimit = 20_A;
        climberConfig.StatorCurrentLimit = 120_A;
        climberConfig.TriggerThreshold = 30_A;
        climberConfig.TriggerThresholdTime = 0.5_s;
        climberConfig.ClosedLoopRampRate = 0.05_s;
        climberConfig.PIDConfigs.WithKP(0.0).WithKI(0.0);

        return climberConfig;
    }

};
