// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L2Command.h"

frc2::CommandPtr L2Command(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::L2Position),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::L2Position);

            }),

            arm->setArmCommand(ArmConstants::ArmL2Reef, ArmConstants::WristL2Reef), frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(ArmConstants::ArmL2Reef, ArmConstants::WristL2Reef);
            })
    );
}
