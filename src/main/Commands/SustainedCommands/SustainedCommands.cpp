// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SustainedCommands.h"

frc2::CommandPtr SustainedCommands(StateManager *stateManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::InitialPosition, stateManager->InitialToSustained()}, std::pair {Positions::Intake,
                    stateManager->IntakeToCoralHold()}, std::pair {Positions::AlgaeLowReef,
                    stateManager->AlgaeLowReefToSustained()}, std::pair {Positions::AlgaeHighReef,
                    stateManager->AlgaeHighReefToSustained()}, std::pair {Positions::AlgaeGround,
                    stateManager->AlgaeGroundToSustained()});
}

frc2::CommandPtr SustainedConfirmedCommands(StateManager *stateManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::L1Confirm, stateManager->L1ConfirmToSustained()}, std::pair {Positions::FrontConfirm,
                    stateManager->FrontConfirmToSustained()}, std::pair {Positions::BackConfirm,
                    stateManager->BackConfirmToSustained()}, std::pair {Positions::NetConfirm,
                    stateManager->NetConfirmToSustained()}, std::pair {Positions::ProcessorConfirm,
                    stateManager->ProcessorConfirmToSustained()});
}
