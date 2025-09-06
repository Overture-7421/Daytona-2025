// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeReefCommand.h"

frc2::CommandPtr AlgaeReefCommand(StateManager* stateManager, AlignManager* alignManager) {
	// return frc2::cmd::Select < AlgaePose > ([alignManager] {
	//     return alignManager->getAlgaePose();
	// },
	// std::pair {AlgaePose::Up, stateManager->setStatePosition(Positions::AlgaeHighReef)}, std::pair {AlgaePose::Down,
	//         stateManager->setStatePosition(Positions::AlgaeLowReef)}

	// );
	return frc2::cmd::None();

}
