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
                    stateManager->IntakeToSustained()}, std::pair {Positions::AlgaeLowReef,
                    stateManager->AlgaeLowReefToSustained()}, std::pair {Positions::AlgaeHighReef,
                    stateManager->AlgaeHighReefToSustained()}, std::pair {Positions::AlgaeGround,
                    stateManager->AlgaeGroundToSustained()}, std::pair {Positions::L1Confirm,
                    stateManager->L1ConfirmToSustained()}, std::pair {Positions::L2FrontConfirm,
                    stateManager->FrontConfirmToSustained()}, std::pair {Positions::L3FrontConfirm,
                    stateManager->FrontConfirmToSustained()}, std::pair {Positions::L4FrontConfirm,
                    stateManager->FrontConfirmToSustained()}, std::pair {Positions::L2BackConfirm,
                    stateManager->BackConfirmToSustained()}, std::pair {Positions::L3BackConfirm,
                    stateManager->BackConfirmToSustained()}, std::pair {Positions::L4BackConfirm,
                    stateManager->BackConfirmToSustained()}, std::pair {Positions::NetConfirm,
                    stateManager->NetConfirmToSustained()}, std::pair {Positions::ProcessorConfirm,
                    stateManager->ProcessorConfirmToSustained()});
}
