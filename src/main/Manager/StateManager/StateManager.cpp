// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StateManager.h"
#include <frc/smartdashboard/SmartDashboard.h>

StateManager::StateManager(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber, Climber *climber,
        AlignManager *alignManager) {
    this->intake = intake;
    this->arm = arm;
    this->elevator = elevator;
    this->grabber = grabber;
    this->climber = climber;
    this->alignManager = alignManager;

    frc::SmartDashboard::PutBoolean("StateManager/IsFinished", false);
}

Positions StateManager::getStatePosition() {
    return state;
}

// bool StateManager::getExecute() {
// 	return execute;
// }

// void StateManager::setExecute(bool value) {
// 	this->execute = value;
// }

// frc2::CommandPtr StateManager::setNewState() {
// 	return frc2::cmd::RunOnce([this]() {
// 		this->state = desiredState;
// 	}
// 	);
// }

// frc2::CommandPtr StateManager::setStatePosition() {
//     return frc2::cmd::Either(frc2::cmd::None(), frc2::cmd::Defer([this] {
//         return this->transitionsMap[this->currentTransitionIndex].commandGenerator();
//     }, {}).AlongWith(setNewState()), [this] {
//         return this->currentTransitionIndex == -1;
//     }
//     ).BeforeStarting([this]() {
//         return frc::SmartDashboard::PutBoolean("StateManager/IsFinished", false);
//     }).FinallyDo([this] {
//         frc::SmartDashboard::PutBoolean("StateManager/IsFinished", true);
//         setExecute(false);
//     });
// }

frc2::CommandPtr StateManager::setStatePosition(Positions desiredState) {
    return frc2::cmd::RunOnce([this, desiredState] {
        this->state = desiredState;
    });
}

frc2::CommandPtr StateManager::setStateOverride() {
    return frc2::cmd::RunOnce([this] {
        this->state = Positions::SustainedPosition;
    });
}

void StateManager::Periodic() {
    frc::SmartDashboard::PutString("StateManager/CurrentState", std::to_string(static_cast<int>(state)));
    // frc::SmartDashboard::PutBoolean("StateManager/Execute", execute);

    // if (execute == false) {
    // for (const Transitions& transitions : transitionsMap) {
    // 	frc::SmartDashboard::PutBoolean("StateManager/CurrentTransition", transitions.currentState == this->state);
    // 	frc::SmartDashboard::PutNumber("StateManager/CurrentIndex", currentTransitionIndex);
    // 	if (transitions.currentState == this->state && transitions.check()) {
    // 		this->desiredState = transitions.nextState;

    // 		this->currentTransitionIndex = &transitions - &transitionsMap[0];
    // 		execute = true;
    // 		break; // Exit loop once valid transition is found
    // 	}
    // }
    // }

}
