// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr SourceCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Select < IntakeStates
            > ([intake] {
                return intake->getState();
            },
            std::pair {IntakeStates::HoldCoral, frc2::cmd::Parallel(
                    elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                            ElevatorConstants::CoralStationPosition).ToPtr(),
                    intake->setIntakeCommand(IntakeConstants::CoralGrab, IntakeConstants::JawCoralOpen, IntakeStates::EnterCoral).FinallyDo(
                            [=]() {
                                intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral);
                            })

            )}, std::pair {IntakeStates::EnterAlgae, frc2::cmd::Parallel(
                    elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                            ElevatorConstants::CoralStationPosition).ToPtr(),
                    intake->setIntakeCommand(IntakeConstants::CoralGrab, IntakeConstants::JawCoralOpen, IntakeStates::EnterCoral).FinallyDo(
                            [=]() {
                                intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral);
                            })

            )}

            );

}
