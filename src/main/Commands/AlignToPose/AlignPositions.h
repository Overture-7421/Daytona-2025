// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include <map>
#include "Enums/ReefSide.h"
#include "Commands/UtilityFunctions/ReefOffset.h"

static const ReefOffset defaultReefOffset {-0.30_m, 0.0_m, 0.50_m, 0.0_deg};

static std::map<ReefLocation, ReefOffset> alignPositionsMap;

static const std::map<ReefLocation, ReefOffset> alignInRed = {};

static const std::map<ReefLocation, ReefOffset> alignInBlue = {};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

