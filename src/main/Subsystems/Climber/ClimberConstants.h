#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

#include "Enums/Positions.h"

struct ClimberConstants {

    constexpr static const units::degree_t ClimberRangeError = 3.0_deg;

    constexpr static const units::degree_t ClosedServo = 122_deg; // Not defined
    constexpr static const units::degree_t OpenedServo = 122_deg; //Not defined

    //61.8 toda la partida
    //124.6 para escalar
    //233.2 empieza
    //Falta poner la posicion donde ya escala/escalo/cerrado
    inline static const std::map<Positions, units::degree_t> ClimberPositions = { {Positions::AlgaeGround, 61.8_deg}, {
            Positions::AlgaeHold, 61.8_deg}, {Positions::AlgaeHighReef, 61.8_deg}, {Positions::AlgaeLowReef, 61.8_deg},
            {Positions::CoralAndAlgae, 61.8_deg}, {Positions::CoralHold, 61.8_deg}, {Positions::InitialPosition,
                    61.8_deg}, {Positions::Intake, 61.8_deg}, {Positions::IntakeCoralStation, 61.8_deg}, {
                    Positions::L1Confirm, 61.8_deg}, {Positions::L1Position, 61.8_deg}, {Positions::L2Back, 61.8_deg}, {
                    Positions::L2BackConfirm, 61.8_deg}, {Positions::L2Front, 61.8_deg}, {Positions::L2FrontConfirm,
                    61.8_deg}, {Positions::L3Back, 61.8_deg}, {Positions::L3BackConfirm, 61.8_deg}, {Positions::L3Front,
                    61.8_deg}, {Positions::L3FrontConfirm, 61.8_deg}, {Positions::L4Back, 61.8_deg}, {
                    Positions::L4BackConfirm, 61.8_deg}, {Positions::L4Front, 61.8_deg}, {Positions::L4FrontConfirm,
                    61.8_deg}, {Positions::NetPosition, 61.8_deg}, {Positions::NetConfirm, 61.8_deg}, {
                    Positions::ProcessorPosition, 61.8_deg}, {Positions::ProcessorConfirm, 61.8_deg}, {
                    Positions::SustainedPosition, 61.8_deg}, {Positions::EndPosition, 124.6_deg}};

    constexpr static const units::degrees_per_second_t ClimberVelocity = 60.0_deg_per_s;
    constexpr static const units::degrees_per_second_squared_t ClimberAcceleration = 30.0_deg_per_s_sq;

    constexpr static const double ClimberEncoderOffset = 0.0;
    constexpr static const double ClimberSensorToMechanism = 64;

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
        climberConfig.PIDConfigs.WithKP(0.0);

        return climberConfig;
    }

};
