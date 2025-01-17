// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L1Command.h"

frc2::CommandPtr L1Command(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::L1Position),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::L1Position);

            }),

            arm->setArmCommand(Constants::ArmL1Reef, Constants::WristL1Reef), frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(Constants::ArmL1Reef, Constants::WristL1Reef);
            })
    );
}
