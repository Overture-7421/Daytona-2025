// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L3Command.h"

frc2::CommandPtr L3Command(StateManager* stateManager, AlignManager* alignManager) {
	// return frc2::cmd::Select < Heading > ([alignManager] {
	//     return alignManager->getHeading();
	// },
	// std::pair {Heading::Back, stateManager->setStatePosition(Positions::L3Back)}, std::pair {Heading::Front,
	//         stateManager->setStatePosition(Positions::L3Front)}

	// );

	return frc2::cmd::None();

}
