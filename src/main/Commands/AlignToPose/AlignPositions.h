// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/AlignToPose/AlignToPose.h"

constexpr static const frc::Pose2d aLeft = { {3.950_m, 2.913_m}, {60_deg}};
constexpr static const frc::Pose2d aCenter = { {3.988_m, 2.894_m}, {60_deg}};
constexpr static const frc::Pose2d aRight = { {4.210_m, 2.754_m}, {60_deg}};

constexpr static const frc::Pose2d bLeft = { {3.247_m, 3.927_m}, {0.0_deg}};
constexpr static const frc::Pose2d bCenter = { {3.237_m, 3.830_m}, {0.0_deg}};
constexpr static const frc::Pose2d bRight = { {3.247_m, 3.625_m}, {0.0_deg}};

constexpr static const frc::Pose2d cLeft = { {3.812_m, 5.068_m}, {-60_deg}};
constexpr static const frc::Pose2d cCenter = { {3.764_m, 5.039_m}, {-60_deg}};
constexpr static const frc::Pose2d cRight = { {3.510_m, 4.893_m}, {-60_deg}};

constexpr static const frc::Pose2d dLeft = { {5.041_m, 5.136_m}, {-120_deg}};
constexpr static const frc::Pose2d dCenter = { {4.953_m, 5.185_m}, {-120_deg}};
constexpr static const frc::Pose2d dRight = { {4.748_m, 5.312_m}, {-120_deg}};

constexpr static const frc::Pose2d eLeft = { {5.733_m, 4.113_m}, {-180_deg}}; 
constexpr static const frc::Pose2d eCenter = { {5.733_m, 4.171_m}, {-180_deg}};
constexpr static const frc::Pose2d eRight = { {5.713_m, 4.434_m}, {-180_deg}};

constexpr static const frc::Pose2d fLeft = { {5.148_m, 2.972_m}, {120_deg}};
constexpr static const frc::Pose2d fCenter = { {5.245_m, 3.030_m}, {120_deg}};
constexpr static const frc::Pose2d fRight = { {5.460_m, 3.147_m}, {120_deg}};

constexpr static const frc::Pose2d stationLeft = { {1.666_m, 7.400_m}, {-54_deg}};
constexpr static const frc::Pose2d stationRight = { {1.582_m, 0.662_m}, {54_deg}};

constexpr static const frc::Pose2d processor = { {6.123_m, 0.495_m}, {90_deg}};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

