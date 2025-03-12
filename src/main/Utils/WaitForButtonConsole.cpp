// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "WaitForButtonConsole.h"

frc2::CommandPtr WaitForButtonConsole(frc2::CommandGenericHID *gamepad, int buttonNumber) {
	return frc2::cmd::WaitUntil([=]() {
		return gamepad->GetHID().GetRawButton(buttonNumber);
	});
}