// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ReefCommands.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr L1Command(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::L1),
                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L1Position),
                            ArmMotion(elevator, arm, ArmConstants::ArmL1Reef, ArmConstants::WristClosed,
                                    ElevatorConstants::L1Position).ToPtr(),
                            ArmMotion(elevator, arm, ArmConstants::ArmL1Reef, ArmConstants::WristL1Reef,
                                    ElevatorConstants::L1Position).ToPtr()

                            ))}

            );

}

frc2::CommandPtr L2Command(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::L2),
                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L2Position),
                            ArmMotion(elevator, arm, ArmConstants::ArmL2Reef, ArmConstants::WristL2Reef,
                                    ElevatorConstants::L2Position).ToPtr()))}

            );

}

frc2::CommandPtr L3Command(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::L3),
                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L3Position),
                            ArmMotion(elevator, arm, ArmConstants::ArmL3Reef, ArmConstants::WristL3Reef,
                                    ElevatorConstants::L3Position).ToPtr()))}

            );

}

frc2::CommandPtr L4Command(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::L4),
                    frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L4Position),
                            ArmMotion(elevator, arm, ArmConstants::ArmL4Reef, ArmConstants::WristL4Reef,
                                    ElevatorConstants::L4Position).ToPtr()))}

            );

}
