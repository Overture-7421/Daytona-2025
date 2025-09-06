// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ConfirmCommand.h"

frc2::CommandPtr ConfirmCommand(StateManager* stateManager) {
	// return frc2::cmd::Select < Positions
	//         > ([stateManager] {
	//             return stateManager->getStatePosition();
	//         },
	//         std::pair {Positions::L1Position, stateManager->setStatePosition(Positions::L1Confirm)}, std::pair {
	//                 Positions::NetPosition, stateManager->setStatePosition(Positions::NetConfirm)}, std::pair {
	//                 Positions::L2Back, stateManager->setStatePosition(Positions::BackConfirm)}, std::pair {
	//                 Positions::L3Back, stateManager->setStatePosition(Positions::BackConfirm)}, std::pair {
	//                 Positions::L4Back, stateManager->setStatePosition(Positions::BackConfirm)}, std::pair {
	//                 Positions::L2Front, stateManager->setStatePosition(Positions::FrontConfirm)}, std::pair {
	//                 Positions::L3Front, stateManager->setStatePosition(Positions::FrontConfirm)}, std::pair {
	//                 Positions::L4Front, stateManager->setStatePosition(Positions::FrontConfirm)}, std::pair {
	//                 Positions::ProcessorPosition, stateManager->setStatePosition(Positions::ProcessorConfirm)}

	//         );

	return frc2::cmd::None();


}
