// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowAlgae.h"
#include "Subsystems/Arm/Constants.h"
#include "Subsystems/Elevator/Constants.h"

frc2::CommandPtr LowAlgae(Arm *arm, Elevator *elevator, OverXboxController *overxboxController) {
    return frc2::cmd::Sequence(elevator->setElevatorCommand(ElevatorConstants::BottomAlgaePosition),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::BottomAlgaePosition);
            }),
            frc2::cmd::Parallel(arm->setArmCommand(Constants::ArmL2Reef, Constants::WristL2Reef)),

            frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(Constants::ArmL2Reef, Constants::WristL2Reef);

            })

    );
}
