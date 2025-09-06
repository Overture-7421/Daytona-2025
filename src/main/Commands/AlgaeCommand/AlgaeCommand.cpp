// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeCommand.h"

frc2::CommandPtr AlgaeCommand(StateManager* stateManager) {
	// return frc2::cmd::Select < Positions > ([stateManager] {
	//     return stateManager->getStatePosition();
	// },
	// std::pair {Positions::SustainedPosition, stateManager->setStatePosition(Positions::AlgaeGround)}, std::pair {
	//         Positions::CoralAndAlgae, stateManager->setStatePosition(Positions::NetPosition)}, std::pair {
	//         Positions::AlgaeHold, stateManager->setStatePosition(Positions::NetPosition)}

	// );

	return frc2::cmd::None();

}
