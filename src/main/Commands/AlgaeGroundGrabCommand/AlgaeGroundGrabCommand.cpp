// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeGroundGrabCommand.h"

frc2::CommandPtr AlgaeGroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Select < IntakeStates
            > ([intake] {
                return intake->getState();
            },
            std::pair {IntakeStates::EnterCoral, frc2::cmd::Parallel(
                    elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                            ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),

                    intake->setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae, IntakeStates::EnterAlgae).FinallyDo([=]() {
                        intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawAlgae, IntakeStates::HoldAlgae);
                    })
            )}, std::pair {IntakeStates::HoldCoral, frc2::cmd::Parallel(
                    elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                            ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),

                    intake->setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae, IntakeStates::EnterAlgae).FinallyDo([=]() {
                        intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawAlgae, HoldAlgae);
                    })
            )});
}
