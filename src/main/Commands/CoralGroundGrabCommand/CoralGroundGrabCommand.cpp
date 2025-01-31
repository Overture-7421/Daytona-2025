// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CoralGroundGrabCommand.h"

frc2::CommandPtr CoralGroundGrabCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralGroundGrabPosition),
            ArmMotion(elevator, arm, ArmConstants::ArmCoralGround, ArmConstants::WristCoralGround,
                    ElevatorConstants::CoralGroundGrabPosition).ToPtr(),

            intake->moveIntake(IntakeConstants::CoralGrab).FinallyDo([=]() {
                intake->moveIntake(IntakeConstants::StopIntake);
            }
            )
    );
}
