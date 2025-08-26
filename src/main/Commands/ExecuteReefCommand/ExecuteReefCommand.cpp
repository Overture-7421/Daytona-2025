// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ExecuteReefCommand.h"

frc2::CommandPtr ExecuteReefCommand(StateManager *stateManager) {
return frc2::cmd::Select< Positions > ([stateManager] {
            return stateManager->getState();
        },
        std::pair {Positions::L2Back, stateManager->setState(Positions::L2Back)},
        std::pair {Positions::L2Front, stateManager->setState(Positions::L2Front)},
        std::pair {Positions::L3Back, stateManager->setState(Positions::L3Back)},
        std::pair {Positions::L4Back, stateManager->setState(Positions::L4Back)},
        std::pair {Positions::L4Front, stateManager->setState(Positions::L4Front)},
        std::pair {Positions::AlgaeHighReef, stateManager->setState(Positions::AlgaeHighReef)},
        std::pair {Positions::AlgaeLowReef, stateManager->setState(Positions::AlgaeLowReef)}

);

}
