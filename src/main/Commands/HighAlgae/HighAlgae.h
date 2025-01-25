// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/DigitalInput.h>
#include <frc2/command/Commands.h>
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Chassis/Chassis.h"


frc2::CommandPtr HighAlgae(Arm *arm, Elevator *elevator, Chassis *chassis);


