// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CoralGroundGrabCommand.h"

frc2::CommandPtr CoralGroundGrabCommandFront(Arm *arm, Elevator *elevator, Intake *intake,
        SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterCoralGround),
                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPositionFront),
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::CoralGroundGrab),
                                    ArmMotion(elevator, arm, ArmConstants::ArmCoralGroundFront,
                                            ArmConstants::WristCoralGroundFront,
                                            ElevatorConstants::CoralGroundGrabPositionFront).ToPtr()))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterCoralStation, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterCoralGround),
                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPositionFront),
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::CoralGroundGrab),
                                    ArmMotion(elevator, arm, ArmConstants::ArmCoralGroundFront,
                                            ArmConstants::WristCoralGroundFront,
                                            ElevatorConstants::CoralGroundGrabPositionFront).ToPtr()))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterAlgaeGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterCoralGround),
                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPositionFront),
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::CoralGroundGrab),
                                    ArmMotion(elevator, arm, ArmConstants::ArmCoralGroundFront,
                                            ArmConstants::WristCoralGroundFront,
                                            ElevatorConstants::CoralGroundGrabPositionFront).ToPtr()))).Until([intake] {
                return intake->isCoralIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldCoral)),
                            frc2::cmd::Sequence(arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))));

}

frc2::CommandPtr CoralGroundGrabCommandBack(Arm *arm, Elevator *elevator, Intake *intake,
        SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterCoralGround),
                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPositionBack),
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::CoralGroundGrab),
                                    ArmMotion(elevator, arm, ArmConstants::ArmCoralGroundBack,
                                            ArmConstants::WristCoralGroundBack,
                                            ElevatorConstants::CoralGroundGrabPositionBack).ToPtr()))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterCoralStation, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterCoralGround),
                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPositionBack),
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::CoralGroundGrab),
                                    ArmMotion(elevator, arm, ArmConstants::ArmCoralGroundBack,
                                            ArmConstants::WristCoralGroundBack,
                                            ElevatorConstants::CoralGroundGrabPositionBack).ToPtr()))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterAlgaeGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterCoralGround),
                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPositionBack),
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::CoralGroundGrab),
                                    ArmMotion(elevator, arm, ArmConstants::ArmCoralGroundBack,
                                            ArmConstants::WristCoralGroundBack,
                                            ElevatorConstants::CoralGroundGrabPositionBack).ToPtr()))).Until([intake] {
                return intake->isCoralIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldCoral)),
                            frc2::cmd::Sequence(arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))));

}
