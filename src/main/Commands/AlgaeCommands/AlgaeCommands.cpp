// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeCommands.h"

frc2::CommandPtr AlgaeReefCommand(StateManager *stateManager, AlignManager *alignManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager, alignManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::SustainedPosition, frc2::cmd::Select < AlgaePose > ([alignManager, stateManager] {
                return alignManager->getAlgaePose();
            },
            std::pair {AlgaePose::Up, stateManager->SustainedToAlgaeHighReef()}, std::pair {AlgaePose::Down,
                    stateManager->SustainedToAlgaeLowReef()})});
}

frc2::CommandPtr AlgaeHighManualCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::SustainedPosition, stateManager->SustainedToAlgaeHighReef()});
}

frc2::CommandPtr AlgaeLowManualCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::SustainedPosition, stateManager->SustainedToAlgaeLowReef()});
}

frc2::CommandPtr AlgaeGroundCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::SustainedPosition, stateManager->SustainedToAlgaeGround()});
}

frc2::CommandPtr AlgaeHoldCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::AlgaeHighReef, stateManager->AlgaeHighReefToAlgaeHold()}, std::pair {
                    Positions::AlgaeLowReef, stateManager->AlgaeLowReefToAlgaeHold()}, std::pair {
                    Positions::AlgaeGround, stateManager->AlgaeGroundToAlgaeHold()});
}

frc2::CommandPtr NetCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::AlgaeHold, stateManager->AlgaeHoldToNet()});
}

frc2::CommandPtr ProcessorCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::AlgaeHold, stateManager->AlgaeHoldToProcessor()});
}
