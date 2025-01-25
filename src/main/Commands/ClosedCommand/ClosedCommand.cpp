// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr ClosedCommand(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(
            ArmMotion(elevator, arm, ArmConstants::ArmClosed, ArmConstants::WristClosed,
                    ElevatorConstants::ClosedPosition).ToPtr(),
            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition), frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::ClosedPosition);

            })

    );
}
