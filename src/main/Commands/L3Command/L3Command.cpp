// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L3Command.h"

frc2::CommandPtr L3Command(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::L3Position),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::L3Position);

            }),

            ArmMotion(elevator, arm, ArmConstants::ArmL3Reef, ArmConstants::WristL3Reef, ElevatorConstants::L3Position).ToPtr()
    );
}
