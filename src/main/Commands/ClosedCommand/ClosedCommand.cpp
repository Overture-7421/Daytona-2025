// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ClosedCommand.h"

frc2::CommandPtr ClosedCommand(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber, Climber *climber) { // HIghALgae
    return frc2::cmd::Parallel(elevator->setCharacterization(1.19_m), arm->setCharacterization(-90_deg));

}
