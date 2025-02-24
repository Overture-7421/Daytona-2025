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
                            frc2::cmd::Sequence(
                                    frc2::cmd::Sequence(
                                            superStructure->setState(SuperStructureStates::EnterCoralGround),
                                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
                                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround,
                                                    ArmConstants::WristCoralGround,
                                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr()),
                                    intake->setIntakeCommand(IntakeConstants::CoralGrab,
                                            IntakeConstants::JawCoralOpen))).Until([intake] {
                        return intake->isCoralIn(IntakeConstants::JawCoralOpen);
                    })}, std::pair {SuperStructureStates::EnterCoralStation,
                    frc2::cmd::RepeatingSequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Sequence(
                                            superStructure->setState(SuperStructureStates::EnterCoralGround),
                                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
                                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround,
                                                    ArmConstants::WristCoralGround,
                                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr()),
                                    intake->setIntakeCommand(IntakeConstants::CoralGrab,
                                            IntakeConstants::JawCoralOpen))).Until([intake] {
                        return intake->isCoralIn(IntakeConstants::JawCoralOpen);
                    })}, std::pair {SuperStructureStates::EnterAlgaeGround,
                    frc2::cmd::RepeatingSequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Sequence(
                                            superStructure->setState(SuperStructureStates::EnterCoralGround),
                                            elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
                                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround,
                                                    ArmConstants::WristCoralGround,
                                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr()),
                                    intake->setIntakeCommand(IntakeConstants::CoralGrab,
                                            IntakeConstants::JawCoralOpen))).Until([intake] {
                        return intake->isCoralIn(IntakeConstants::JawCoralOpen);
                    })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(
                                    intake->setIntakeCommand(IntakeConstants::StopIntake,
                                            IntakeConstants::JawCoralClose),
                                    superStructure->setState(SuperStructureStates::HoldCoral)),
                            frc2::cmd::Sequence(arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))));

}
