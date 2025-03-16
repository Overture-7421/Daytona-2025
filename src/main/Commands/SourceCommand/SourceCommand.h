// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Intake/Intake.h"
#include <frc2/command/Commands.h>
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "OvertureLib/Utils/WaitForButton/WaitForButton.h"
#include "Utils/WaitForButtonConsole.h"

frc2::CommandPtr SourceCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure,
        OverXboxController *gamepad);
frc2::CommandPtr SourceCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure,
        frc2::CommandGenericHID *gamepadConsole);
frc2::CommandPtr SourceCommandAuto(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure);
