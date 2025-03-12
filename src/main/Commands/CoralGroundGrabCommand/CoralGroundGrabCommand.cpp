// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CoralGroundGrabCommand.h"

frc2::CommandPtr CoralGroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral,
                    frc2::cmd::RepeatingSequence(
                                    frc2::cmd::Parallel(
                                            superStructure->setState(SuperStructureStates::EnterCoralGround),
                                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
                                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround,
                                                    ArmConstants::WristCoralGround,
                                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr()),
                                    intake->moveIntake(IntakeConstants::CoralGroundGrab)).Until([intake] {
                        return intake->isCoralIn();
                    })}, std::pair {SuperStructureStates::EnterCoralStation,
                    frc2::cmd::RepeatingSequence(
                                    frc2::cmd::Parallel(
                                            superStructure->setState(SuperStructureStates::EnterCoralGround),
                                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
                                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround,
                                                    ArmConstants::WristCoralGround,
                                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr()),
                                    intake->moveIntake(IntakeConstants::CoralGroundGrab)).Until([intake] {
                        return intake->isCoralIn();
                    })}, std::pair {SuperStructureStates::EnterAlgaeGround,
                    frc2::cmd::RepeatingSequence(
                                    frc2::cmd::Parallel(
                                            superStructure->setState(SuperStructureStates::EnterCoralGround),
                                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
                                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround,
                                                    ArmConstants::WristCoralGround,
                                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr()),
                                    intake->moveIntake(IntakeConstants::CoralGroundGrab)).Until([intake] {
                        return intake->isCoralIn();
                    })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldCoral)),
                            frc2::cmd::Sequence(arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))));

}
