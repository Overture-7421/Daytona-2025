// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ArmConstants {
	constexpr static const units::degree_t ArmAngleRange = 1.7_deg;
	constexpr static const units::degree_t WristAngleRange = 1.7_deg; //Bajar tolerancia

	constexpr static const units::degree_t ArmScoreL2 = 30_deg;
	constexpr static const units::degree_t ArmScoreL3 = 25_deg;
	constexpr static const units::degree_t ArmScoreL4 = 32_deg;

	constexpr static const units::degree_t ArmCoralInter = 75_deg; //15
	constexpr static const units::degree_t ArmAlgaeInter = 60_deg;

	constexpr static const units::degree_t ArmL1Reef = 28.0_deg; //58
	constexpr static const units::degree_t WristL1Reef = -90.0_deg; //90

	constexpr static const units::degree_t ArmL2Reef = 73.0_deg; //73
	constexpr static const units::degree_t WristL2Reef = 0.0_deg;

	constexpr static const units::degree_t ArmL3Reef = 73.0_deg; //73
	constexpr static const units::degree_t WristL3Reef = 0.0_deg;

	constexpr static const units::degree_t ArmL4Reef = 73.0_deg; //73
	constexpr static const units::degree_t WristL4Reef = 0.0_deg;

	constexpr static const units::degree_t ArmL4ReefAuto = 48.0_deg; //57
	constexpr static const units::degree_t WristL4ReefAuto = 0.0_deg;

	constexpr static const units::degree_t ArmLowAlgae = 8.0_deg; //2
	constexpr static const units::degree_t WristLowAlgae = 90.0_deg;

	constexpr static const units::degree_t ArmHighAlgae = 8.0_deg; //2
	constexpr static const units::degree_t WristHighAlgae = 90.0_deg;

	constexpr static const units::degree_t ArmCoralStation = 108_deg; //130
	constexpr static const units::degree_t ArmCoralStationAway = 90_deg; //95
	constexpr static const units::degree_t WristCoralStation = 90.0_deg;

	constexpr static const units::degree_t ArmProcessor = 197.0_deg; //90
	constexpr static const units::degree_t WristProcessor = 90.0_deg;

	constexpr static const units::degree_t ArmNet = 90.0_deg; // 1
	constexpr static const units::degree_t WristNet = 0.0_deg;

	constexpr static const units::degree_t ArmCoralGroundFront = -24.0_deg; //Back 201
	constexpr static const units::degree_t WristCoralGroundFront = -90.0_deg; // Back 90

	constexpr static const units::degree_t ArmCoralGroundBack = 207.0_deg; //Back 201
	constexpr static const units::degree_t WristCoralGroundBack = 90.0_deg; // Back 90

	constexpr static const units::degree_t ArmAlgaeGround = -20.0_deg; //110
	constexpr static const units::degree_t WristAlgaeGround = -90.0_deg;

	constexpr static const units::degree_t ArmClosed = 90.0_deg; // 0
	constexpr static const units::degree_t WristClosed = 0.0_deg;

	constexpr static const double ArmRotorToSensor = 75.0;
	constexpr static const double WristRotorToSensor = 40.5;

	constexpr static const units::turns_per_second_t ArmCruiseVelocity = 120_tps;
	constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 120_tr_per_s_sq;
	constexpr static const units::turns_per_second_t WristCruiseVelocity = 50.0_tps;
	constexpr static const units::turns_per_second_squared_t WristCruiseAcceleration = 40_tr_per_s_sq;

	constexpr static const units::turns_per_second_t SlowVelocity = 3_tps;
	constexpr static const units::turns_per_second_squared_t SlowAccelerationa = 1_tr_per_s_sq;

	constexpr static const double ArmCANCoderId = 25;
	constexpr static const double WristCANCoderId = 27;

	constexpr static const double ArmLeftMotorId = 23;

	constexpr static const double WristMotorId = 26;

	constexpr static const OverTalonFXConfig ArmLeftConfig() {
		OverTalonFXConfig armLeftConfig;
		armLeftConfig.MotorId = ArmLeftMotorId;
		armLeftConfig.NeutralMode = ControllerNeutralMode::Brake;
		armLeftConfig.useFOC = true;
		armLeftConfig.Inverted = true;

		armLeftConfig.ClosedLoopRampRate = 0.05_s;
		armLeftConfig.CurrentLimit = 30_A;
		armLeftConfig.StatorCurrentLimit = 120_A;
		armLeftConfig.TriggerThreshold = 40_A;
		armLeftConfig.TriggerThresholdTime = 0.5_s;
		armLeftConfig.PIDConfigs.GravityType = 1;
		armLeftConfig.PIDConfigs.WithKG(0.3).WithKV(1).WithKP(80); //KV1.9 P150

		return armLeftConfig;
	}

	constexpr static const OverTalonFXConfig WristConfig() {
		OverTalonFXConfig wristConfig;
		wristConfig.MotorId = WristMotorId;
		wristConfig.NeutralMode = ControllerNeutralMode::Brake;
		wristConfig.useFOC = true;
		//wristConfig.Inverted = true;

		wristConfig.ClosedLoopRampRate = 0.05_s;
		wristConfig.CurrentLimit = 25_A;
		wristConfig.StatorCurrentLimit = 120_A;
		wristConfig.TriggerThreshold = 30_A;
		wristConfig.TriggerThresholdTime = 0.5_s;
		wristConfig.PIDConfigs.WithKV(1.8).WithKP(120); //KV1.8 P120

		return wristConfig;
	}

	constexpr static const CanCoderConfig ArmCANConfig() {
		CanCoderConfig armCANConfig;
		armCANConfig.CanCoderId = ArmCANCoderId;
		armCANConfig.Offset = 0.35791015625_tr;

		return armCANConfig;
	}

	constexpr static const CanCoderConfig WristCANConfig() {
		CanCoderConfig wristCANConfig;
		wristCANConfig.CanCoderId = WristCANCoderId;
		wristCANConfig.Offset = -0.449462890625_tr;
		wristCANConfig.SensorDirection = 1;

		return wristCANConfig;
	}

};
