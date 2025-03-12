// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/AlignToPose/AlignToPose.h"

constexpr static const frc::Pose2d aLeft = { {3.926_m, 2.821_m}, {60_deg}};
constexpr static const frc::Pose2d aCenter = { {4.056_m, 2.679_m}, {60_deg}};
constexpr static const frc::Pose2d aRight = { {4.220_m, 2.640_m}, {60_deg}};

constexpr static const frc::Pose2d bLeft = { {3.160_m, 3.920_m}, {0.0_deg}};
constexpr static const frc::Pose2d bCenter = { {3.130_m, 3.723_m}, {0.0_deg}};
constexpr static const frc::Pose2d bRight = { {3.160_m, 3.560_m}, {0.0_deg}};

constexpr static const frc::Pose2d cLeft = { {3.725_m, 5.127_m}, {-60_deg}};
constexpr static const frc::Pose2d cCenter = { {3.549_m, 5.088_m}, {-60_deg}};
constexpr static const frc::Pose2d cRight = { {3.448_m, 4.963_m}, {-60_deg}};

constexpr static const frc::Pose2d dLeft = { {5.058_m, 5.229_m}, {-120_deg}};
constexpr static const frc::Pose2d dCenter = { {4.953_m, 5.332_m}, {-120_deg}};
constexpr static const frc::Pose2d dRight = { {4.775_m, 5.396_m}, {-120_deg}};

constexpr static const frc::Pose2d eLeft = { {5.816_m, 4.132_m}, {-180_deg}};
constexpr static const frc::Pose2d eCenter = { {5.889_m, 4.327_m}, {-180_deg}};
constexpr static const frc::Pose2d eRight = { {5.816_m, 4.463_m}, {-180_deg}};

constexpr static const frc::Pose2d fLeft = { {5.245_m, 2.923_m}, {120_deg}};
constexpr static const frc::Pose2d fCenter = { {5.431_m, 2.972_m}, {120_deg}};
constexpr static const frc::Pose2d fRight = { {5.519_m, 3.099_m}, {120_deg}};

constexpr static const frc::Pose2d stationLeft = { {0.809_m, 6.810_m}, {-54_deg}};
constexpr static const frc::Pose2d stationRight = { {0.806_m, 1.220_m}, {54_deg}};

constexpr static const frc::Pose2d processor = { {5.665_m, 0.661_m}, {-90_deg}};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

