// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlgaeGroundGrabCommand.h"

frc2::CommandPtr AlgaeGroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::AlgaeGroundGrabPosition),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::AlgaeGroundGrabPosition);

            })
            ,

            ArmMotion(elevator, arm, ArmConstants::ArmAlgaeGround, ArmConstants::WristAlgaeGround,
                    ElevatorConstants::AlgaeGroundGrabPosition).ToPtr(),

            intake->moveIntake(IntakeConstants::AlgaeGrab).FinallyDo([=]() {
                intake->moveIntake(IntakeConstants::StopIntake);
            }
            )
    );
}
