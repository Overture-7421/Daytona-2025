// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Intake/Intake.h"
#include <frc2/command/Commands.h>
#include "Subsystems/SuperStructure/SuperStructure.h"

frc2::CommandPtr SpitGamePiece(Intake *intake, SuperStructure *superStructure);
