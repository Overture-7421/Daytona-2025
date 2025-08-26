// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeReefCommand.h"

frc2::CommandPtr AlgaeReefCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getState();
    },
    std::pair {Positions::AlgaeHighReef, stateManager->setState(Positions::AlgaeHighReef)}, std::pair {
            Positions::AlgaeLowReef, stateManager->setState(Positions::AlgaeLowReef)}

    );

}
