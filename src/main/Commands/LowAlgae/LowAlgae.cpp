// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowAlgae.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr LowAlgae(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::EnterHighAlgae, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::HoldAlgae, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::SpitCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(frc2::cmd::Wait(0.8_s),
                            superStructure->setState(SuperStructureStates::HoldAlgae),
                            arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)));

}

frc2::CommandPtr LowAlgaeAuto(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::EnterHighAlgae, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::HoldAlgae, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::SpitCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(
                            frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                    elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                                            ElevatorConstants::LowAlgae).ToPtr(),
                                    superStructure->setScoringState(SuperStructureScoringStates::SpitAlgaeState),
                                    intake->moveIntake(IntakeConstants::AlgaeGrab)))).Until([intake] {
                return intake->isAlgaeIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(frc2::cmd::Wait(0.8_s),
                            superStructure->setState(SuperStructureStates::HoldAlgae)));

}
