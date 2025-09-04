// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CharacterizationCommand.h"

frc2::CommandPtr CharacterizationCommand(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber,
        Climber *climber) {
    return frc2::cmd::Sequence(elevator->setCharacterization(0.0_m), arm->setCharacterization(0.0_deg),
            intake->setCharacterization(0.0_V, 0.0_V, 0.0_deg), grabber->setCharacterization(0.0_V));

}
