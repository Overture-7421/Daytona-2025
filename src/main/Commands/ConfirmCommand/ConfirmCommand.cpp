// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ConfirmCommand.h"

frc2::CommandPtr ConfirmCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager] {
                return stateManager->getStatePosition();
            },
            std::pair {Positions::L1Position, stateManager->L1PositionToL1Confirm()}, std::pair {Positions::NetPosition,
                    stateManager->NetPositionToNetConfirm()}, std::pair {Positions::L2Back,
                    stateManager->L2BackToBackConfirm()}, std::pair {Positions::L3Back,
                    stateManager->L3BackToBackConfirm()}, std::pair {Positions::L4Back,
                    stateManager->L4BackToBackConfirm()}, std::pair {Positions::L2Front,
                    stateManager->L2FrontToFrontConfirm()}, std::pair {Positions::L3Front,
                    stateManager->L3FrontToFrontConfirm()}, std::pair {Positions::L4Front,
                    stateManager->L4FrontToFrontConfirm()}, std::pair {Positions::ProcessorPosition,
                    stateManager->ProcessorPositionToProcessorConfirm()}, std::pair {Positions::L4FrontAuto,
                    stateManager->L4FrontAutoToFrontAutoConfirm()}, std::pair {Positions::L4BackAuto,
                    stateManager->L4BackAutoToFrontAutoConfirm()},

            std::pair {Positions::AlgaeHold, stateManager->L1ClosedConfirm()});
}
