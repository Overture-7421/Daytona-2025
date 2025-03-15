// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPositions.h"

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, frc2::cmd::Sequence(AlignToPose(chassis, aTarget, true).ToPtr(),
                    AlignToPose(chassis, aLeft, false).ToPtr())}, std::pair {ReefLocation::B, frc2::cmd::Sequence(
                    AlignToPose(chassis, bTarget, true).ToPtr(), AlignToPose(chassis, bLeft, false).ToPtr())}, std::pair {
                    ReefLocation::C, frc2::cmd::Sequence(AlignToPose(chassis, cTarget, true).ToPtr(),
                            AlignToPose(chassis, cLeft, false).ToPtr())}, std::pair {ReefLocation::D,
                    frc2::cmd::Sequence(AlignToPose(chassis, dTarget, true).ToPtr(),
                            AlignToPose(chassis, dLeft, false).ToPtr())}, std::pair {ReefLocation::E,
                    frc2::cmd::Sequence(AlignToPose(chassis, eTarget, true).ToPtr(),
                            AlignToPose(chassis, eLeft, false).ToPtr())}, std::pair {ReefLocation::F,
                    frc2::cmd::Sequence(AlignToPose(chassis, fTarget, true).ToPtr(),
                            AlignToPose(chassis, fLeft, false).ToPtr())});
}

frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aCenter, false).ToPtr()}, std::pair {ReefLocation::B,
                    AlignToPose(chassis, bCenter, false).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis,
                    cCenter, false).ToPtr()}, std::pair {ReefLocation::D, AlignToPose(chassis, dCenter, false).ToPtr()}, std::pair {
                    ReefLocation::E, AlignToPose(chassis, eCenter, false).ToPtr()}, std::pair {ReefLocation::F,
                    AlignToPose(chassis, fCenter, false).ToPtr()});
}

frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, frc2::cmd::Sequence(AlignToPose(chassis, aTarget, true).ToPtr(),
                    AlignToPose(chassis, aRight, false).ToPtr())}, std::pair {ReefLocation::B, frc2::cmd::Sequence(
                    AlignToPose(chassis, bTarget, true).ToPtr(), AlignToPose(chassis, bRight, false).ToPtr())}, std::pair {
                    ReefLocation::C, frc2::cmd::Sequence(AlignToPose(chassis, cTarget, true).ToPtr(),
                            AlignToPose(chassis, cRight, false).ToPtr())}, std::pair {ReefLocation::D,
                    frc2::cmd::Sequence(AlignToPose(chassis, dTarget, true).ToPtr(),
                            AlignToPose(chassis, dRight, false).ToPtr())}, std::pair {ReefLocation::E,
                    frc2::cmd::Sequence(AlignToPose(chassis, eTarget, true).ToPtr(),
                            AlignToPose(chassis, eRight, false).ToPtr())}, std::pair {ReefLocation::F,
                    frc2::cmd::Sequence(AlignToPose(chassis, fTarget, true).ToPtr(),
                            AlignToPose(chassis, fRight, false).ToPtr())});
}

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return frc2::cmd::Select < StationLocation > ([chassis, tagLayout]() {
        return findClosestStationLocation(chassis, tagLayout);
    },
    std::pair {StationLocation::Left, AlignToPose(chassis, stationLeft, false).ToPtr()}, std::pair {
            StationLocation::Right, AlignToPose(chassis, stationRight, false).ToPtr()});
}

frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    return AlignToPose(chassis, processor, false).ToPtr();
}
