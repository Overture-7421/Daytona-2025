// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeCommand.h"

frc2::CommandPtr AlgaeCommand(StateManager *stateManager) {
return frc2::cmd::Select< Positions > ([stateManager] {
            return stateManager->getState();
        },
        std::pair {Positions::AlgaeGround, stateManager->setState(Positions::AlgaeGround)},
        std::pair {Positions::NetPosition, stateManager->setState(Positions::NetPosition)},

);

}
