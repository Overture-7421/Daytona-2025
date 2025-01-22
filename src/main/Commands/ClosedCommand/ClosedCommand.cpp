// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"

frc2::CommandPtr ClosedCommand(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Sequence(arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
            frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(ArmConstants::ArmClosed, ArmConstants::WristClosed);
            }),
            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition), frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::ClosedPosition);

            })

    );
}
