// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr ClosedCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Select < IntakeStates
            > ([intake] {
                return intake->getState();
            },
            std::pair {IntakeStates::EnterCoral, frc2::cmd::Parallel(
                    intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral),
                    ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                            ElevatorConstants::ClosedPosition).ToPtr(),
                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))}, std::pair {
                    IntakeStates::EnterAlgae, frc2::cmd::Parallel(
                            intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))}, std::pair {
                    IntakeStates::SpitCoral, frc2::cmd::Parallel(
                            intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))}, std::pair {
                    IntakeStates::SpitAlgae, frc2::cmd::Parallel(
                            intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))},
                            std::pair{IntakeStates::HoldCoral, frc2::cmd::Parallel(
                                intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralClose, IntakeStates::HoldCoral),
                            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                                    ElevatorConstants::ClosedPosition).ToPtr(),
                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)
                            )}

            );

}
