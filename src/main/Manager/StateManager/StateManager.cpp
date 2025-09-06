// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StateManager.h"
#include <frc/smartdashboard/SmartDashboard.h>

StateManager::StateManager(Intake* intake, Arm* arm, Elevator* elevator, Grabber* grabber, Climber* climber,
	AlignManager* alignManager, OverXboxController* driver, OverXboxController* oprtr, OverConsole* console,
	frc2::Trigger* endToInitial) {
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

bool StateManager::getExecute() {
	return execute;
}

bool StateManager::setExecute(bool value) {
	execute = value;
	return execute;
}

frc2::CommandPtr StateManager::setStatePosition() {
	return frc2::cmd::Either(
		frc2::cmd::None(),
		frc2::cmd::Defer([this] {
		return this->transitionsMap[this->currentTransitionIndex].commandGenerator();
	}, {}),
		[this] { return this->currentTransitionIndex == -1; }
	).AndThen([this] {
		setExecute(false);
	});
}

frc2::CommandPtr StateManager::setStateOverride() {
	return frc2::cmd::RunOnce([this] {
		this->state = Positions::SustainedPosition;
	});
}

void StateManager::Periodic() {
	frc::SmartDashboard::PutString("StateManager/CurrentState", std::to_string(static_cast<int>(state)));
	frc::SmartDashboard::PutBoolean("StateManager/Execute", execute);

	if (execute == false) {
		for (const Transitions& transitions : transitionsMap) {
			frc::SmartDashboard::PutBoolean("StateManager/CurrentTransition", transitions.currentState == this->state);
			frc::SmartDashboard::PutNumber("StateManager/CurrentIndex", currentTransitionIndex);
			frc::SmartDashboard::PutNumber("StateManager/Test", test);
			if (transitions.currentState == this->state && transitions.check()) {
				this->state = transitions.nextState;

				this->currentTransitionIndex = &transitions - &transitionsMap[0];
				test++;
				execute = true;
				break; // Exit loop once valid transition is found
			}
		}
	}

}
