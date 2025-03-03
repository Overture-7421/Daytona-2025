// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L4Command.h"

frc2::CommandPtr L4Command(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    /*
                     frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::L4Position),
                     ArmMotion(elevator, arm, ArmConstants::ArmCoralInter, ArmConstants::WristL4Reef,
                     ElevatorConstants::L4Position).ToPtr()),
                     */

                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L4Position),
                            ArmMotion(elevator, arm, ArmConstants::ArmL4Reef, ArmConstants::WristL4Reef,
                                    ElevatorConstants::L4Position).ToPtr()))}

            );

}

frc2::CommandPtr L4AutoCommand(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(

                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L4PositionAuto),
                            ArmMotion(elevator, arm, ArmConstants::ArmL4ReefAuto, ArmConstants::WristL4ReefAuto,
                                    ElevatorConstants::L4PositionAuto).ToPtr()))}

            );

}
