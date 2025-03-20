// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/AlignToPose/AlignToPose.h"

constexpr static const frc::Pose2d aLeft = { {3.890_m, 2.840_m}, {60_deg}};
constexpr static const frc::Pose2d aCenter = { {4.056_m, 2.679_m}, {60_deg}};
constexpr static const frc::Pose2d aRight = { {4.197_m, 2.647_m}, {60_deg}};
constexpr static const frc::Pose2d aTarget = { {4.019_m, 2.584_m}, {60_deg}};

constexpr static const frc::Pose2d bLeft = { {3.163_m, 3.928_m}, {0.0_deg}};
constexpr static const frc::Pose2d bCenter = { {3.130_m, 3.723_m}, {0.0_deg}};
constexpr static const frc::Pose2d bRight = { {3.163_m, 3.560_m}, {0.0_deg}};
constexpr static const frc::Pose2d bTarget = { {2.842_m, 3.7_m}, {0.0_deg}};

constexpr static const frc::Pose2d cLeft = { {3.725_m, 5.127_m}, {-60_deg}};
constexpr static const frc::Pose2d cCenter = { {3.549_m, 5.088_m}, {-60_deg}};
constexpr static const frc::Pose2d cRight = { {3.448_m, 4.963_m}, {-60_deg}};
constexpr static const frc::Pose2d cTarget = { {3.43_m, 5.276_m}, {-60_deg}};

constexpr static const frc::Pose2d dLeft = { {5.068_m, 5.219_m}, {-120_deg}};
constexpr static const frc::Pose2d dCenter = { {4.953_m, 5.332_m}, {-120_deg}};
constexpr static const frc::Pose2d dRight = { {4.775_m, 5.396_m}, {-120_deg}};
constexpr static const frc::Pose2d dTarget = { {5.056_m, 5.566_m}, {-120_deg}};

constexpr static const frc::Pose2d eLeft = { {5.816_m, 4.000_m}, {-180_deg}};
constexpr static const frc::Pose2d eCenter = { {5.889_m, 4.327_m}, {-180_deg}};
constexpr static const frc::Pose2d eRight = { {5.816_m, 4.463_m}, {-180_deg}};
constexpr static const frc::Pose2d eTarget = { {6.134_m, 4.3_m}, {-180_deg}};

constexpr static const frc::Pose2d fLeft = { {5.230_m, 2.908_m}, {120_deg}};
constexpr static const frc::Pose2d fCenter = { {5.431_m, 2.972_m}, {120_deg}};
constexpr static const frc::Pose2d fRight = { {5.519_m, 3.099_m}, {120_deg}};
constexpr static const frc::Pose2d fTarget = { {5.524_m, 2.774_m}, {120_deg}};

constexpr static const frc::Pose2d stationLeft = { {0.809_m, 6.810_m}, {-54_deg}};
constexpr static const frc::Pose2d stationRight = { {0.806_m, 1.220_m}, {54_deg}};

constexpr static const frc::Pose2d processor = { {5.706_m, 1.022_m}, {-130_deg}};

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);
frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);
frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);
frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);

frc2::CommandPtr leftAlignAuto(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);
frc2::CommandPtr rightAlignAuto(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed);

