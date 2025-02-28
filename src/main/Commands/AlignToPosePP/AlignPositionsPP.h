// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Commands/AlignToPosePP/AlignToPosePP.h"
#include "Commands/AlignToPose/AlignToPose.h"

constexpr static const pathplanner::PathConstraints constraints = pathplanner::PathConstraints(4.5_mps, 4.0_mps_sq,
        540_deg_per_s, 720_deg_per_s_sq);

constexpr static const frc::Pose2d aPose = { {4.290_m, 2.338_m}, {60_deg}};
constexpr static const frc::Pose2d bPose = { {2.896_m, 3.382_m}, {0_deg}};
constexpr static const frc::Pose2d cPose = { {3.251_m, 5.137_m}, {-60_deg}};
constexpr static const frc::Pose2d dPose = { {4.719_m, 5.702_m}, {-120_deg}};
constexpr static const frc::Pose2d ePose = { {6.094_m, 4.678_m}, {-180_deg}};
constexpr static const frc::Pose2d fPose = { {5.860_m, 2.962_m}, {120_deg}};
constexpr static const frc::Pose2d stationPoseLeft = { {1.745_m, 7.281_m}, {-54_deg}};
constexpr static const frc::Pose2d stationPoseRight = { {1.774_m, 0.759_m}, {54_deg}};
constexpr static const frc::Pose2d processorPose = { {5.665_m, 0.661_m}, {-90_deg}};

constexpr static const frc::Pose2d center = { {0.10_m, 0_m}, {90_deg}};
constexpr static const frc::Pose2d left = { {0.25_m, 0_m}, {0_deg}};
constexpr static const frc::Pose2d right = { {0.26_m, -0.05_m}, {116.06_deg}};

frc2::CommandPtr leftAlignPP(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout,
        photon::PhotonCamera *frontLeftCamera);
frc2::CommandPtr centerAlignPP(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout,
        photon::PhotonCamera *frontLeftCamera);
frc2::CommandPtr rightAlignPP(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout,
        photon::PhotonCamera *frontLeftCamera);

