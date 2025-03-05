// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpitL1.h"

frc2::CommandPtr SpitL1(Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    superStructure->setState(SuperStructureStates::SpitCoral),
                    intake->moveIntake(IntakeConstants::StopIntake), intake->moveIntake(IntakeConstants::CoralSpit))}

            );

}
