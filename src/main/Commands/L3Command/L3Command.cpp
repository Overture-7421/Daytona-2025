// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L3Command.h"

frc2::CommandPtr L3Command(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    /*
                     frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::L3Position),
                     ArmMotion(elevator, arm, ArmConstants::ArmCoralInter, ArmConstants::WristL3Reef,
                     ElevatorConstants::L3Position).ToPtr()),
                     */

                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L3Position),
                            ArmMotion(elevator, arm, ArmConstants::ArmL3Reef, ArmConstants::WristL3Reef,
                                    ElevatorConstants::L3Position).ToPtr()))}

            );

}
