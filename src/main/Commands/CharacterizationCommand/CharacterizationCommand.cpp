// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CharacterizationCommand.h"

frc2::CommandPtr CharacterizationCommand(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber,
        Climber *climber) { // AlgaeHigh
    return frc2::cmd::Sequence(grabber->setCharacterization(6_V), intake->setCharacterization(0_V, 0_V, 131_deg),
            elevator->setCharacterization(0.97_m), arm->setCharacterization(90.0_deg),
            elevator->setCharacterization(1.28_m), arm->setCharacterization(0_deg));

}
