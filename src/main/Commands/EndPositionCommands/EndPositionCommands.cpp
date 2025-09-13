// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "EndPositionCommands.h"

frc2::CommandPtr EndPositionCommands(StateManager* stateManager) {
	// return frc2::cmd::Select < Positions >([stateManager] {
	// 	return stateManager->getStatePosition();
	// },
	// 	std::pair{ Positions::SustainedPosition, stateManager->SustainedToEndPosition() }, std::pair{ Positions::AlgaeHold,
	// 			stateManager->AlgaeHoldToEndPosition() }, std::pair{ Positions::Intake,
	// 			stateManager->SustainedToEndPosition() });
	return stateManager->SustainedToEndPosition();
}
