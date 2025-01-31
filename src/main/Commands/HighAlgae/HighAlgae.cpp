// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "HighAlgae.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr HighAlgae(Arm *arm, Elevator *elevator) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::HighAlgae),

            ArmMotion(elevator, arm, ArmConstants::ArmHighAlgae, ArmConstants::WristHighAlgae,
                    ElevatorConstants::HighAlgae).ToPtr());
}

