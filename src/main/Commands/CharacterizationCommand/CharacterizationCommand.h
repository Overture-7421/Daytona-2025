// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Climber/Climber.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Grabber/Grabber.h"
#include "Subsystems/Intake/Intake.h"

frc2::CommandPtr CharacterizationCommand(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber,
        Climber *climber);
