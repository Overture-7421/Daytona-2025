// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowAlgae.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr LowAlgae(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::LowAlgae);
            })
            ,
            ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae,
                    ElevatorConstants::LowAlgae).ToPtr());
}
