// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/AlignToPose/AlignToPose.h"

constexpr static const frc::Pose2d aLeft = { {3.704_m, 2.988_m}, {60_deg}};
constexpr static const frc::Pose2d aCenter = { {3.836_m, 2.892_m}, {60_deg}};
constexpr static const frc::Pose2d aRight = { {3.980_m, 2.820_m}, {60_deg}};

constexpr static const frc::Pose2d bLeft = { {3.201_m, 4.187_m}, {0.0_deg}};
constexpr static const frc::Pose2d bCenter = { {3.213_m, 4.019_m}, {0.0_deg}};
constexpr static const frc::Pose2d bRight = { {3.189_m, 3.851_m}, {0.0_deg}};

constexpr static const frc::Pose2d cLeft = { {3.968_m, 5.230_m}, {-60_deg}};
constexpr static const frc::Pose2d cCenter = { {3.848_m, 5.146_m}, {-60_deg}};
constexpr static const frc::Pose2d cRight = { {3.704_m, 5.050_m}, {-60_deg}};

constexpr static const frc::Pose2d dLeft = { {5.287_m, 5.074_m}, {-120_deg}};
constexpr static const frc::Pose2d dCenter = { {5.155_m, 5.146_m}, {-120_deg}};
constexpr static const frc::Pose2d dRight = { {4.999_m, 5.230_m}, {-120_deg}};

constexpr static const frc::Pose2d eLeft = { {5.802_m, 3.875_m}, {-180_deg}};
constexpr static const frc::Pose2d eCenter = { {5.802_m, 4.019_m}, {-180_deg}};
constexpr static const frc::Pose2d eRight = { {5.802_m, 4.187_m}, {-180_deg}};

constexpr static const frc::Pose2d fLeft = { {4.987_m, 2.820_m}, {120_deg}};

constexpr static const frc::Pose2d fCenter = { {5.131_m, 2.904_m}, {120_deg}};
constexpr static const frc::Pose2d fRight = { {5.287_m, 3.000_m}, {120_deg}};

constexpr static const frc::Pose2d stationLeft = { {1.666_m, 7.400_m}, {126_deg}};
constexpr static const frc::Pose2d stationRight = { {1.582_m, 0.662_m}, {-126_deg}};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

