// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowAlgae.h"

frc2::CommandPtr LowAlgae(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::LowAlgae);
            }),
            arm->setArmCommand(ArmConstants::ArmLowAlgae, ArmConstants::WristLowhAlgae),

            frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(ArmConstants::ArmLowAlgae, ArmConstants::WristLowhAlgae);

            })

    );
}
