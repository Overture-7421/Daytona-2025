// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SuperStructure.h"

SuperStructure::SuperStructure() = default;

frc2::CommandPtr SuperStructure::setState(SuperStructureStates state) {
    frc::SmartDashboard::PutNumber("States/TargetState", state);
    return this->RunOnce([this, state] {
        this->state = state;
    });

}

SuperStructureStates SuperStructure::getState() {
    frc::SmartDashboard::PutNumber("States/CurrentState", state);
    return state;
}

void SuperStructure::Periodic() {
    frc::SmartDashboard::PutNumber("States/State", state);
}
