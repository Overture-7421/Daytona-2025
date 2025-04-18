// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "NetCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"

frc2::CommandPtr NetCommand(Arm *arm, Elevator *elevator, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldAlgae, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::NetState),
                    elevator->setElevatorCommand(ElevatorConstants::NetPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmNet, ArmConstants::WristNet,
                            ElevatorConstants::NetPosition).ToPtr())},

            std::pair {SuperStructureStates::SpitAlgae, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::NetState),
                    elevator->setElevatorCommand(ElevatorConstants::NetPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmNet, ArmConstants::WristNet,
                            ElevatorConstants::NetPosition).ToPtr())},

            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    superStructure->setScoringState(SuperStructureScoringStates::NetState),
                    elevator->setElevatorCommand(ElevatorConstants::NetPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmNet, ArmConstants::WristNet,
                            ElevatorConstants::NetPosition).ToPtr())}

            );

}
