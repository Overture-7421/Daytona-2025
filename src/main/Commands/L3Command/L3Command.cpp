// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L3Command.h"

frc2::CommandPtr L3Command(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getState();
    },
    std::pair {Positions::L3Back, stateManager->setState(Positions::L3Back)}, std::pair {Positions::L3Front,
            stateManager->setState(Positions::L3Front)}

    );

}
