// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L4Command.h"

frc2::CommandPtr L4Command(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::L4Position),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::L4Position);

            }),

            ArmMotion(elevator, arm, ArmConstants::ArmL4Reef, ArmConstants::WristL4Reef, ElevatorConstants::L4Position).ToPtr()
    );
}

