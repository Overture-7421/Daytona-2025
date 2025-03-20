// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/Commands.h>

frc2::CommandPtr WaitForButtonConsole(frc2::CommandGenericHID *gamepad, int buttonNumber);
