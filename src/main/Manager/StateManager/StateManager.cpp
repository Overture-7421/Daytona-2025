// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StateManager.h"

StateManager::StateManager() = default;

Positions StateManager::getState() {
    return state;
}

frc2::CommandPtr StateManager::setState(Positions state) {
    return frc2::cmd::RunOnce([this, state] {
        this->state = state;
    });

}
