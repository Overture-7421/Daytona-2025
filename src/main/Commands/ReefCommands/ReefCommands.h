// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>
#include "Manager/StateManager/StateManager.h"
#include "Manager/AlignManager/AlignManager.h"

frc2::CommandPtr L1Command(StateManager *stateManager);
frc2::CommandPtr PassCommand(StateManager *stateManager);
frc2::CommandPtr PassCommandAlign(StateManager *stateManager);
frc2::CommandPtr L2Command(StateManager *stateManager, AlignManager *alignManager);
frc2::CommandPtr L3Command(StateManager *stateManager, AlignManager *alignManager);
frc2::CommandPtr L4Command(StateManager *stateManager, AlignManager *alignManager);
frc2::CommandPtr L4CommandAuto(StateManager *stateManager);
