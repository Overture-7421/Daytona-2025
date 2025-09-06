// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CharacterizationCommand.h"

frc2::CommandPtr CharacterizationCommand(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber,
        Climber *climber) {
    return frc2::cmd::Sequence(elevator->setCharacterization(1.05_m), arm->setCharacterization(90.0_deg));

}
