// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StateManager.h"
#include <frc/smartdashboard/SmartDashboard.h>

StateManager::StateManager(Intake* intake, Arm* arm, Elevator* elevator, Grabber* grabber, Climber* climber, AlignManager* alignManager,
	OverXboxController* driver, OverXboxController* oprtr, OverConsole* console, frc2::Trigger* endToInitial) {
	this->intake = intake;
	this->arm = arm;
	this->elevator = elevator;
	this->grabber = grabber;
	this->climber = climber;
	this->driver = driver;
	this->oprtr = oprtr;
	this->console = console;
	this->endToInitial = endToInitial;
	this->alignManager = alignManager;
}

Positions StateManager::getStatePosition() {
	return state;
}

frc2::CommandPtr StateManager::setStatePosition(Positions desiredState) {
	// Find the match
	for (const Transitions& transitions : transitionsMap) {
		if (transitions.currentState == this->state && transitions.nextState == desiredState
			&& transitions.check()) {
			// Execute the transition command first, then update state after completion
			return transitions.commandPtr().AndThen(
				frc2::cmd::RunOnce([this, desiredState] {
				this->state = desiredState;
			})
			);
		}
	}

	// If no valid transition is found, do nothing
	return frc2::cmd::None();
}

frc2::CommandPtr StateManager::setStateOverride() {
	return frc2::cmd::RunOnce([this] {
		this->state = Positions::SustainedPosition;
	});
}

void StateManager::Periodic() {
	frc::SmartDashboard::PutString("StateManager/State", std::to_string(static_cast<int>(state)));
}
