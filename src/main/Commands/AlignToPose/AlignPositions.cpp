// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPositions.h"
#include "pathplanner/lib/util/FlippingUtil.h"

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aLeft).ToPtr()}, std::pair {ReefLocation::B, AlignToPose(
                    chassis, bLeft).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis, cLeft).ToPtr()}, std::pair {
                    ReefLocation::D, AlignToPose(chassis, dLeft).ToPtr()}, std::pair {ReefLocation::E, AlignToPose(
                    chassis, eLeft).ToPtr()}, std::pair {ReefLocation::F, AlignToPose(chassis, fLeft).ToPtr()});
}

frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aCenter).ToPtr()}, std::pair {ReefLocation::B, AlignToPose(
                    chassis, bCenter).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis, cCenter).ToPtr()}, std::pair {
                    ReefLocation::D, AlignToPose(chassis, dCenter).ToPtr()}, std::pair {ReefLocation::E, AlignToPose(
                    chassis, eCenter).ToPtr()}, std::pair {ReefLocation::F, AlignToPose(chassis, fCenter).ToPtr()});
}

frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aRight).ToPtr()}, std::pair {ReefLocation::B, AlignToPose(
                    chassis, bRight).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis, cRight).ToPtr()}, std::pair {
                    ReefLocation::D, AlignToPose(chassis, dRight).ToPtr()}, std::pair {ReefLocation::E, AlignToPose(
                    chassis, eRight).ToPtr()}, std::pair {ReefLocation::F, AlignToPose(chassis, fRight).ToPtr()});
}

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < StationLocation > ([chassis, tagLayout]() {
        return findClosestStationLocation(chassis, tagLayout);
    },
    std::pair {StationLocation::Left, AlignToPose(chassis, stationLeft).ToPtr()}, std::pair {StationLocation::Right,
            AlignToPose(chassis, stationRight).ToPtr()});
}

frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return AlignToPose(chassis, processor).ToPtr();
}
