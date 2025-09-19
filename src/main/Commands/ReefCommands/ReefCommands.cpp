// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ReefCommands.h"

frc2::CommandPtr L1Command(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::Intake, stateManager->IntakeToL1Position()});
}

frc2::CommandPtr PassCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions > ([stateManager] {
        return stateManager->getStatePosition();
    },
    std::pair {Positions::L1Position, stateManager->IntakeToCoralHold()}, std::pair {Positions::SustainedPosition,
            stateManager->CoralHoldToL1Position()});
}

frc2::CommandPtr L2Command(StateManager *stateManager, AlignManager *alignManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager, alignManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::SustainedPosition, frc2::cmd::Select < Heading
                    > ([alignManager, stateManager] {
                        return alignManager->getHeading();
                    },
                    std::pair {Heading::Back, stateManager->CoralHoldToL2Back()}, std::pair {Heading::Front,
                            stateManager->CoralHoldToL2Front()})}, std::pair {Positions::L3Front,
                    stateManager->CoralHoldToL3Front()}, std::pair {Positions::L4Front,
                    stateManager->CoralHoldToL4Front()});
}

frc2::CommandPtr L3Command(StateManager *stateManager, AlignManager *alignManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager, alignManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::SustainedPosition, frc2::cmd::Select < Heading
                    > ([alignManager, stateManager] {
                        return alignManager->getHeading();
                    },
                    std::pair {Heading::Back, stateManager->CoralHoldToL3Back()}, std::pair {Heading::Front,
                            stateManager->CoralHoldToL3Front()})},

            std::pair {Positions::L2Front, stateManager->CoralHoldToL2Front()}, std::pair {Positions::L4Front,
                    stateManager->CoralHoldToL4Front()});
}

frc2::CommandPtr L4Command(StateManager *stateManager, AlignManager *alignManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager, alignManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::SustainedPosition, frc2::cmd::Select < Heading
                    > ([alignManager, stateManager] {
                        return alignManager->getHeading();
                    },
                    std::pair {Heading::Back, stateManager->CoralHoldToL4Back()}, std::pair {Heading::Front,
                            stateManager->CoralHoldToL4Front()})},

            std::pair {Positions::L3Front, stateManager->CoralHoldToL3Front()}, std::pair {Positions::L2Front,
                    stateManager->CoralHoldToL2Front()});
}

frc2::CommandPtr L4CommandAuto(StateManager *stateManager) {
    return stateManager->InitialToL4Front();
}
