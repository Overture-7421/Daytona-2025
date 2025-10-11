#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

#include "Enums/Positions.h"

struct ClimberConstants {

	// positivo es para adentro
	// negativo es para afuera

	//10-15 angulos reales equivalen a 50 degrees del climber

	//-920 descansa toda la partida
	// -570 horizonte para escalar
	//1600 para escalado

	constexpr static const units::degree_t ClimberRangeError = 1.5_deg;

	constexpr static const units::degree_t ClimberRest = 10_deg;
	constexpr static const units::degree_t ClimberOpen = 70_deg;
	constexpr static const units::degree_t ClimberClosed = 273_deg;

	constexpr static const units::turns_per_second_t ClimberVelocity = 50.0_tps;
	constexpr static const units::turns_per_second_squared_t ClimberAcceleration = 40_tr_per_s_sq;

	constexpr static const double ClimberEncoderOffset = 0.0;
	constexpr static const double ClimberSensorToMechanism = 64;

	constexpr static const double ClimberMotorId = 22;
	constexpr static const double climberCANCoderId = 31;

	constexpr static const CanCoderConfig ClimberCANConfig() {
		CanCoderConfig climberCANConfig;
		climberCANConfig.CanCoderId = climberCANCoderId;
		climberCANConfig.Offset = -0.92578125_tr;
		climberCANConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
		climberCANConfig.absoluteDiscontinuityPoint = 1.0_tr;
		return climberCANConfig;
	}

	constexpr static const OverTalonFXConfig ClimberConfig() {
		OverTalonFXConfig climberConfig;
		climberConfig.MotorId = ClimberMotorId;
		climberConfig.NeutralMode = ControllerNeutralMode::Brake;
		climberConfig.Inverted = false;
		climberConfig.useFOC = true;

		climberConfig.CurrentLimit = 40_A;
		climberConfig.StatorCurrentLimit = 120_A;
		climberConfig.TriggerThreshold = 60_A;
		climberConfig.TriggerThresholdTime = 0.5_s;
		climberConfig.ClosedLoopRampRate = 0.05_s;
		climberConfig.PIDConfigs.WithKP(200.0);

		return climberConfig;
	}

};
