// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include <map>
#include "Enums/ReefSide.h"
#include "Commands/UtilityFunctions/ReefOffset.h"

static ReefOffset defaultReefOffset {0.05_m, -0.05_m, 0.01_m, 0.0_deg};

static std::map<ReefLocation, ReefOffset> alignPositionsMap;

static const std::map<ReefLocation, ReefOffset> alignInRed = {};

static const std::map<ReefLocation, ReefOffset> alignInBlue = {};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
//frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

//frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
//frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

frc2::CommandPtr leftAlignAuto(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignAuto(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

