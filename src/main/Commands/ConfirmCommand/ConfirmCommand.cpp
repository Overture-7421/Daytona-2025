// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ConfirmCommand.h"

frc2::CommandPtr ConfirmCommand(StateManager *stateManager) {
    return frc2::cmd::Select < Positions
            > ([stateManager] {
                return stateManager->getState();
            },
            std::pair {Positions::L1Confirm, stateManager->setState(Positions::L1Confirm)}, std::pair {
                    Positions::NetConfirm, stateManager->setState(Positions::NetConfirm)}, std::pair {
                    Positions::L2BackConfirm, stateManager->setState(Positions::L2BackConfirm)}, std::pair {
                    Positions::L3BackConfirm, stateManager->setState(Positions::L3BackConfirm)}, std::pair {
                    Positions::L4BackConfirm, stateManager->setState(Positions::L4BackConfirm)}, std::pair {
                    Positions::L2FrontConfirm, stateManager->setState(Positions::L2FrontConfirm)}, std::pair {
                    Positions::L3FrontConfirm, stateManager->setState(Positions::L3FrontConfirm)}, std::pair {
                    Positions::L4FrontConfirm, stateManager->setState(Positions::L4FrontConfirm)}, std::pair {
                    Positions::ProcessorConfirm, stateManager->setState(Positions::ProcessorConfirm)}

            );

}
