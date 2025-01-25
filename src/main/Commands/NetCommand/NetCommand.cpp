// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "NetCommand.h"

frc2::CommandPtr NetCommand(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(
            frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::NetPosition),
                    frc2::cmd::WaitUntil([elevator] {
                        return elevator->isElevatorAtPosition(ElevatorConstants::NetPosition);

                    }),

                    arm->setArmCommand(Constants::ArmNet, Constants::WristNet), frc2::cmd::WaitUntil([arm] {
                        return arm->isArmAtPosition(Constants::ArmNet, Constants::WristNet);
                    })
            )
    );
}
