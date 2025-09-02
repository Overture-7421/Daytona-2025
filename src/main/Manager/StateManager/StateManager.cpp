// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StateManager.h"

StateManager::StateManager(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber, Climber *climber,
        OverXboxController *driver, OverXboxController *oprtr, OverConsole *console, frc2::Trigger *endToInitial) {
    this->intake = intake;
    this->arm = arm;
    this->elevator = elevator;
    this->grabber = grabber;
    this->climber = climber;
    this->driver = driver;
    this->oprtr = oprtr;
    this->console = console;
    this->endToInitial = endToInitial;
}

Positions StateManager::getStatePosition() {
    return state;
}

frc2::CommandPtr StateManager::setStatePosition(Positions state) {
    return frc2::cmd::RunOnce([this, state] {
        for (Transitions transitions : transitionsMap) {
            if (transitions.currentState == state && transitions.check()) {
                this->state = state;
                this->commandScheduled = transitions.commandPtr();
            }
        }
    }).AndThen(std::move(commandScheduled)).AndThen([this]() {
        commandScheduled = frc2::cmd::None();
    });

}

frc2::CommandPtr StateManager::setStateOverride() {
    return frc2::cmd::RunOnce([this] {
        this->state = Positions::SustainedPosition;
    });
}
