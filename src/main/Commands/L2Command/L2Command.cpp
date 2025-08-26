// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L2Command.h"

frc2::CommandPtr L2Command(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getState();
    },
    std::pair {Positions::L2Back, stateManager->setState(Positions::L2Back)}, std::pair {Positions::L2Front,
            stateManager->setState(Positions::L2Front)}

    );

}
