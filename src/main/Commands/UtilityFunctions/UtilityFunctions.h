// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Enums/ReefLocation.h"
#include "Enums/StationLocation.h"

#include "Subsystems/Chassis/Chassis.h"
#include <frc/DriverStation.h>
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h"

ReefLocation findClosestReefLocation(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

StationLocation findClosestStationLocation(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
