// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GroundGrabCommand.h"

frc2::CommandPtr GroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::GroundGrabPosition),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::GroundGrabPosition);

            })
            ,

            ArmMotion(elevator, arm, ArmConstants::ArmGround, ArmConstants::WristGround,
                    ElevatorConstants::GroundGrabPosition).ToPtr(),

            intake->moveIntake(IntakeConstants::CoralGrab).FinallyDo([=]() {
                intake->moveIntake(IntakeConstants::StopIntake);
            }
            )
    );
}
