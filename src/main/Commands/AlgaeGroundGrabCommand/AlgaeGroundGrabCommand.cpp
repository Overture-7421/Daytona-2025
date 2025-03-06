// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeGroundGrabCommand.h"

frc2::CommandPtr AlgaeGroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterAlgaeGround),
                            elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                                    ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::AlgaeGrab))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::EnterCoralStation, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterAlgaeGround),
                            elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                                    ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::AlgaeGrab))).Until([intake] {
                return intake->isAlgaeIn();
            })}, std::pair {SuperStructureStates::EnterCoralGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterAlgaeGround),
                            elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                                    ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::AlgaeGrab))).Until([intake] {
                return intake->isAlgaeIn();
            })}).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldAlgae)),
                            frc2::cmd::Sequence(arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))));
}
