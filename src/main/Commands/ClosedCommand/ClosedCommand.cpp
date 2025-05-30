// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr ClosedCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::EnterCoralStation, frc2::cmd::Sequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::HoldCoral),
                            superStructure->setScoringState(SuperStructureScoringStates::HoldBadCoral)),
                    frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)))

            }, std::pair {SuperStructureStates::EnterCoralGround, frc2::cmd::Sequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::HoldCoral),
                            superStructure->setScoringState(SuperStructureScoringStates::HoldBadCoral)),
                    frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)))

            }, std::pair {SuperStructureStates::EnterAlgaeGround, frc2::cmd::Sequence(
                    superStructure->setState(SuperStructureStates::HoldAlgae),
                    ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(),
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(), intake->moveIntake(IntakeConstants::AlgaeHold),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)
                    //intake->moveIntake(IntakeConstants::StopIntake)
                            )

            }, std::pair {SuperStructureStates::EnterLowAlgae, frc2::cmd::Sequence(
                    superStructure->setState(SuperStructureStates::HoldAlgae),
                    ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(),
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(), intake->moveIntake(IntakeConstants::AlgaeHold),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)
                    //intake->moveIntake(IntakeConstants::StopIntake)
                            )

            }, std::pair {SuperStructureStates::EnterHighAlgae, frc2::cmd::Sequence(
                    superStructure->setState(SuperStructureStates::HoldAlgae),
                    ArmMotion(elevator, arm, ArmConstants::ArmHighAlgae, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(),
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(), intake->moveIntake(IntakeConstants::AlgaeHold),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)
                    //intake->moveIntake(IntakeConstants::StopIntake)
                            )

            }, std::pair {SuperStructureStates::SpitCoral, frc2::cmd::Parallel(
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(), intake->moveIntake(IntakeConstants::StopIntake),
                    superStructure->setState(SuperStructureStates::HoldCoral),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))

            }, std::pair {SuperStructureStates::SpitAlgae, frc2::cmd::Parallel(
                    intake->moveIntake(IntakeConstants::StopIntake),
                    superStructure->setState(SuperStructureStates::HoldCoral),
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))

            }, std::pair {SuperStructureStates::HoldCoral, frc2::cmd::Sequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::HoldCoral),
                            superStructure->setScoringState(SuperStructureScoringStates::HoldBadCoral)),
                    frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)))

            }, std::pair {SuperStructureStates::HoldAlgae, frc2::cmd::Sequence(
                    superStructure->setState(SuperStructureStates::HoldAlgae),
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(), intake->moveIntake(IntakeConstants::AlgaeHold),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)
                    //intake->moveIntake(IntakeConstants::StopIntake)
                            )

            }

            ).BeforeStarting([elevator] {
                return elevator->setElevatorLowerSpeed();
            }).AndThen([elevator] {
                return elevator->setElevatorNormalSpeed();
            });

}
