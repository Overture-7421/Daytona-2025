// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPositionsPP.h"

frc2::CommandPtr leftAlignPP(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout,
        photon::PhotonCamera *frontLeftCamera) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aPose).ToPtr()}, std::pair {ReefLocation::B, AlignToPose(
                    chassis, bPose).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis, cPose).ToPtr()}, std::pair {
                    ReefLocation::D, AlignToPose(chassis, dPose).ToPtr()}, std::pair {ReefLocation::E, AlignToPose(
                    chassis, ePose).ToPtr()}, std::pair {ReefLocation::F, AlignToPose(chassis, fPose).ToPtr()}).AndThen(
                    AlignToPosePP(chassis, left, frontLeftCamera).ToPtr());
}

frc2::CommandPtr centerAlignPP(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout,
        photon::PhotonCamera *frontLeftCamera) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aPose).ToPtr()}, std::pair {ReefLocation::B, AlignToPose(
                    chassis, bPose).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis, cPose).ToPtr()}, std::pair {
                    ReefLocation::D, AlignToPose(chassis, dPose).ToPtr()}, std::pair {ReefLocation::E, AlignToPose(
                    chassis, ePose).ToPtr()}, std::pair {ReefLocation::F, AlignToPose(chassis, fPose).ToPtr()}).AndThen(
                    AlignToPosePP(chassis, center, frontLeftCamera).ToPtr());
}

frc2::CommandPtr rightAlignPP(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout,
        photon::PhotonCamera *frontLeftCamera) {
    return AlignToPosePP(chassis, right, frontLeftCamera).ToPtr();
}
