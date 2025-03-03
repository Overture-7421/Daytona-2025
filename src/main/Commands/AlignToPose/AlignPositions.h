// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/AlignToPose/AlignToPose.h"

constexpr static const frc::Pose2d aLeft = { {3.900_m, 2.913_m}, {60_deg}};
constexpr static const frc::Pose2d aCenter = { {4.056_m, 2.679_m}, {60_deg}};
constexpr static const frc::Pose2d aRight = { {4.202_m, 2.757_m}, {60_deg}};

constexpr static const frc::Pose2d bLeft = { {3.237_m, 3.986_m}, {0.0_deg}};
constexpr static const frc::Pose2d bCenter = { {3.130_m, 3.723_m}, {0.0_deg}};
constexpr static const frc::Pose2d bRight = { {3.237_m, 3.655_m}, {0.0_deg}};

constexpr static const frc::Pose2d cLeft = { {3.822_m, 5.078_m}, {-60_deg}};
constexpr static const frc::Pose2d cCenter = { {3.549_m, 5.088_m}, {-60_deg}};
constexpr static const frc::Pose2d cRight = { {3.530_m, 4.922_m}, {-60_deg}};

constexpr static const frc::Pose2d dLeft = { {5.080_m, 5.146_m}, {-120_deg}};
constexpr static const frc::Pose2d dCenter = { {4.953_m, 5.332_m}, {-120_deg}};
constexpr static const frc::Pose2d dRight = { {4.787_m, 5.302_m}, {-120_deg}};

constexpr static const frc::Pose2d eLeft = { {5.733_m, 4.083_m}, {-180_deg}};
constexpr static const frc::Pose2d eCenter = { {5.889_m, 4.327_m}, {-180_deg}};
constexpr static const frc::Pose2d eRight = { {5.713_m, 4.415_m}, {-180_deg}};

constexpr static const frc::Pose2d fLeft = { {5.158_m, 2.962_m}, {120_deg}};
constexpr static const frc::Pose2d fCenter = { {5.431_m, 2.972_m}, {120_deg}};
constexpr static const frc::Pose2d fRight = { {5.440_m, 3.128_m}, {120_deg}};

constexpr static const frc::Pose2d stationLeft = { {1.745_m, 7.281_m}, {-54_deg}};
constexpr static const frc::Pose2d stationRight = { {1.774_m, 0.759_m}, {54_deg}};

constexpr static const frc::Pose2d processor = { {5.665_m, 0.661_m}, {-90_deg}};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);
frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

