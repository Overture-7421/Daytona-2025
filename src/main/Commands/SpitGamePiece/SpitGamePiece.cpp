// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpitGamePiece.h"

frc2::CommandPtr SpitGamePiece(Intake *intake, SuperStructure *superStructure, Elevator *elevator, Arm *arm) {
    return frc2::cmd::Select < SuperStructureScoringStates
            > ([superStructure] {
                return superStructure->getScoringState();
            },
            std::pair {SuperStructureScoringStates::L1, frc2::cmd::Sequence(
                    superStructure->setState(SuperStructureStates::SpitCoral),
                    intake->moveIntake(IntakeConstants::CoralSpitL1),
                    superStructure->setScoringState(SuperStructureScoringStates::DontScore))}, std::pair {
                    SuperStructureScoringStates::L2,
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Parallel(
                                            ArmMotion(elevator, arm, ArmConstants::ArmScoreL2,
                                                    ArmConstants::WristClosed, elevator->getPosition()).ToPtr().WithTimeout(
                                                    2_s), intake->moveIntake(IntakeConstants::CoralSpit)),
                                    elevator->setElevatorCommand(ElevatorConstants::L2SpitPosition)),
                            superStructure->setState(SuperStructureStates::SpitCoral),
                            superStructure->setScoringState(SuperStructureScoringStates::DontScore))}, std::pair {
                    SuperStructureScoringStates::L3,
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Parallel(
                                            ArmMotion(elevator, arm, ArmConstants::ArmScoreL3,
                                                    ArmConstants::WristClosed, elevator->getPosition()).ToPtr().WithTimeout(
                                                    2_s), intake->moveIntake(IntakeConstants::CoralSpit)),
                                    elevator->setElevatorCommand(ElevatorConstants::L3SpitPosition)),
                            superStructure->setState(SuperStructureStates::SpitCoral),
                            superStructure->setScoringState(SuperStructureScoringStates::DontScore))}, std::pair {
                    SuperStructureScoringStates::L4,
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Parallel(
                                            ArmMotion(elevator, arm, ArmConstants::ArmScoreL4,
                                                    ArmConstants::WristClosed, elevator->getPosition()).ToPtr().WithTimeout(
                                                    2_s)
                                                    //intake->moveIntake(IntakeConstants::CoralSpit)
                                                    ), elevator->setElevatorCommand(ElevatorConstants::L4SpitPosition)),
                            superStructure->setState(SuperStructureStates::SpitCoral),
                            superStructure->setScoringState(SuperStructureScoringStates::DontScore))}, std::pair {
                    SuperStructureScoringStates::ProcessorState, frc2::cmd::Sequence(
                            intake->moveIntake(IntakeConstants::AlgaeRelease),
                            superStructure->setState(SuperStructureStates::SpitAlgae),
                            superStructure->setScoringState(SuperStructureScoringStates::DontScore))}, std::pair {
                    SuperStructureScoringStates::NetState, frc2::cmd::Sequence(
                            intake->moveIntake(IntakeConstants::AlgaeRelease),
                            superStructure->setState(SuperStructureStates::SpitAlgae),
                            superStructure->setScoringState(SuperStructureScoringStates::DontScore))},

            std::pair {SuperStructureScoringStates::SpitAlgaeState, frc2::cmd::Sequence(
                    intake->moveIntake(IntakeConstants::AlgaeRelease),
                    superStructure->setState(SuperStructureStates::SpitAlgae),
                    superStructure->setScoringState(SuperStructureScoringStates::DontScore))},

            std::pair {SuperStructureScoringStates::HoldBadCoral, frc2::cmd::Sequence(
                    intake->moveIntake(IntakeConstants::SpitBadCoral),
                    superStructure->setState(SuperStructureStates::SpitCoral),
                    superStructure->setScoringState(SuperStructureScoringStates::DontScore))}

            );

}
