// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>
#include "Manager/StateManager/StateManager.h"

// frc2::CommandPtr AlgaeReefCommand(StateManager *stateManager, AlignManager *alignManager);
frc2::CommandPtr AlgaeHighManualCommand(StateManager *stateManager);
frc2::CommandPtr AlgaeLowManualCommand(StateManager *stateManager);
frc2::CommandPtr AlgaeGroundCommand(StateManager *stateManager);
frc2::CommandPtr AlgaeHoldCommand(StateManager *stateManager);
frc2::CommandPtr NetCommand(StateManager *stateManager);
frc2::CommandPtr ProcessorCommand(StateManager *stateManager);
