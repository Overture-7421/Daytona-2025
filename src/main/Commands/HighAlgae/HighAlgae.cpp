// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "HighAlgae.h"
#include "Subsystems/Arm/Constants.h"
#include "Subsystems/Elevator/Constants.h"

frc2::CommandPtr HighAlgae(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::TopAlgaePosition),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::TopAlgaePosition);
            }),
            arm->setArmCommand(Constants::ArmL3Reef, Constants::WristL3Reef), frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(Constants::ArmL3Reef, Constants::WristL3Reef);

            })

    );
}
