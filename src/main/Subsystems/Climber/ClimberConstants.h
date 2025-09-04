#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

#include "Enums/Positions.h"

struct ClimberConstants {

    constexpr static const units::degree_t ClimberRangeError = 1.0_deg;

    constexpr static const units::degree_t ClosedServo = 10_deg; // Not defined
    constexpr static const units::degree_t OpenedServo = 0.0_deg; //Not defined

    inline static const std::map<Positions, units::degree_t> ClimberPositions = { {Positions::AlgaeGround, 0.0_deg}, {
            Positions::AlgaeHold, 0.0_deg}, {Positions::AlgaeHighReef, 0.0_deg}, {Positions::AlgaeLowReef, 0.0_deg}, {
            Positions::CoralAndAlgae, 0.0_deg}, {Positions::CoralHold, 0.0_deg}, {Positions::InitialPosition, 0.0_deg},
            {Positions::Intake, 0.0_deg}, {Positions::IntakeCoralStation, 0.0_deg}, {Positions::L1Confirm, 0.0_deg}, {
                    Positions::L1Position, 0.0_deg}, {Positions::L2Back, 0.0_deg}, {Positions::L2BackConfirm, 0.0_deg},
            {Positions::L2Front, 0.0_deg}, {Positions::L2FrontConfirm, 0.0_deg}, {Positions::L3Back, 0.0_deg}, {
                    Positions::L3BackConfirm, 0.0_deg}, {Positions::L3Front, 0.0_deg}, {Positions::L3FrontConfirm,
                    0.0_deg}, {Positions::L4Back, 0.0_deg}, {Positions::L4BackConfirm, 0.0_deg}, {Positions::L4Front,
                    0.0_deg}, {Positions::L4FrontConfirm, 0.0_deg}, {Positions::NetPosition, 0.0_deg}, {
                    Positions::NetConfirm, 0.0_deg}, {Positions::ProcessorPosition, 0.0_deg}, {
                    Positions::ProcessorConfirm, 0.0_deg}, {Positions::SustainedPosition, 0.0_deg}, {
                    Positions::EndPosition, 0.0_deg}};

    constexpr static const units::degrees_per_second_t ClimberVelocity = 1.0_deg_per_s;
    constexpr static const units::degrees_per_second_squared_t ClimberAcceleration = 1.0_deg_per_s_sq;

    constexpr static const double ClimberEncoderOffset = 0;
    constexpr static const double ClimberSensorToMechanism = 1;

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
