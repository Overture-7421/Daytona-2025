// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L4Command.h"

frc2::CommandPtr L4Command(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getState();
    },
    std::pair {Positions::L4Back, stateManager->setState(Positions::L4Back)}, std::pair {Positions::L4Front,
            stateManager->setState(Positions::L4Front)}

    );

}
