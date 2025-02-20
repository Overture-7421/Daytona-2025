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
                    frc2::cmd::Parallel(
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround, ArmConstants::WristCoralGround,
                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr(),
                            intake->setIntakeCommand(IntakeConstants::CoralGrab, IntakeConstants::JawCoralOpen),
                            superStructure->setState(SuperStructureStates::EnterCoralGround)).Until([intake] {
                        return intake->isCoralIn(IntakeConstants::JawCoralOpen);
                    }).FinallyDo(
                            [=]() {
                                intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose), superStructure->setState(
                                        SuperStructureStates::HoldCoral);
                            })}, std::pair {SuperStructureStates::EnterCoralStation,
                    frc2::cmd::Parallel(
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround, ArmConstants::WristCoralGround,
                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr(),
                            intake->setIntakeCommand(IntakeConstants::CoralGrab, IntakeConstants::JawCoralOpen),
                            superStructure->setState(SuperStructureStates::EnterCoralGround)).Until([intake] {
                        return intake->isCoralIn(IntakeConstants::JawCoralOpen);
                    }).FinallyDo(
                            [=]() {
                                intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose), superStructure->setState(
                                        SuperStructureStates::HoldCoral);
                            })}, std::pair {SuperStructureStates::EnterAlgaeGround,
                    frc2::cmd::Parallel(
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround, ArmConstants::WristCoralGround,
                                    ElevatorConstants::CoralGroundGrabPosition).ToPtr(),
                            intake->setIntakeCommand(IntakeConstants::CoralGrab, IntakeConstants::JawCoralOpen),
                            superStructure->setState(SuperStructureStates::EnterCoralGround)).Until([intake] {
                        return intake->isCoralIn(IntakeConstants::JawCoralOpen);
                    }).FinallyDo(
                            [=]() {
                                intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose), superStructure->setState(
                                        SuperStructureStates::HoldCoral);
                            })}

            );

}
