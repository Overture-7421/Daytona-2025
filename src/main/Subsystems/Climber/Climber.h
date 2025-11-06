// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Servo.h>

#include "Subsystems/Climber/ClimberConstants.h"

class Climber : public frc2::SubsystemBase {
public:
	Climber();

	void setOffset();

	void setToAngle(units::degree_t climberAngle);
	frc2::CommandPtr setClimberCommand(units::degree_t climberAngle);

	frc2::CommandPtr setClimberClimbedCommand(units::degree_t climberAngle);
	frc2::CommandPtr disableClimberCommand();
	bool isClimberAtPosition(units::degree_t climberAngle);

	void Periodic() override;

private:

	OverTalonFX climberMotor{ ClimberConstants::ClimberConfig(), "rio" };
	//OverCANCoder climberCANCoder{ ClimberConstants::ClimberCANConfig(), "rio" };

	MotionMagicVoltage armVoltage{ 0_tr };
	bool isDisabled = true;

	units::degree_t offset = 0.0_deg;
	units::degree_t target = 233_deg; //aquí se pone la posición inicial (NO definido aun)

	frc::Servo servo{ 9 }; // Not defined yet

};
