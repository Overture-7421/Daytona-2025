// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L2Command.h"

frc2::CommandPtr L2Command(StateManager *stateManager, AlignManager *alignManager) {
    // return frc2::cmd::Select < Heading > ([alignManager] {
    //     return alignManager->getHeading();
    // },
    // std::pair {Heading::Back, stateManager->setStatePosition(Positions::L2Back)}, std::pair {Heading::Front,
    //         stateManager->setStatePosition(Positions::L2Front)}

    // );

    return frc2::cmd::None();

}
