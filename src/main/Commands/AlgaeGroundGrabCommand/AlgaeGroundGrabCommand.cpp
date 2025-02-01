// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeGroundGrabCommand.h"

frc2::CommandPtr AlgaeGroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Select < IntakeStates
            > ([intake] {
                return intake->getState();
            },
            std::pair {IntakeStates::EnterCoral, frc2::cmd::Parallel(intake->setState(IntakeStates::EnterAlgae),
                    elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                            ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),

                    intake->setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae).FinallyDo([=]() {
                        intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawAlgae);
                        intake->setState(IntakeStates::HoldAlgae);
                    })
            )}, std::pair {IntakeStates::HoldCoral, frc2::cmd::Parallel(intake->setState(IntakeStates::EnterAlgae),
                    elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
                    ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                            ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),

                    intake->setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae).FinallyDo([=]() {
                        intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawAlgae);
                        intake->setState(IntakeStates::HoldAlgae);
                    })
            )});
}
