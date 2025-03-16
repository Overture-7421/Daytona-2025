// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPositions.h"

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, frc2::cmd::Sequence(AlignToPose(chassis, aTarget, true).ToPtr(),
                    AlignToPose(chassis, aLeft, iAmSpeed).ToPtr())}, std::pair {ReefLocation::B, frc2::cmd::Sequence(
                    AlignToPose(chassis, bTarget, true).ToPtr(), AlignToPose(chassis, bLeft, iAmSpeed).ToPtr())}, std::pair {
                    ReefLocation::C, frc2::cmd::Sequence(AlignToPose(chassis, cTarget, true).ToPtr(),
                            AlignToPose(chassis, cLeft, iAmSpeed).ToPtr())}, std::pair {ReefLocation::D,
                    frc2::cmd::Sequence(AlignToPose(chassis, dTarget, true).ToPtr(),
                            AlignToPose(chassis, dLeft, iAmSpeed).ToPtr())}, std::pair {ReefLocation::E,
                    frc2::cmd::Sequence(AlignToPose(chassis, eTarget, true).ToPtr(),
                            AlignToPose(chassis, eLeft, iAmSpeed).ToPtr())}, std::pair {ReefLocation::F,
                    frc2::cmd::Sequence(AlignToPose(chassis, fTarget, true).ToPtr(),
                            AlignToPose(chassis, fLeft, iAmSpeed).ToPtr())});
}

frc2::CommandPtr centerAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, AlignToPose(chassis, aCenter, iAmSpeed).ToPtr()}, std::pair {ReefLocation::B,
                    AlignToPose(chassis, bCenter, iAmSpeed).ToPtr()}, std::pair {ReefLocation::C, AlignToPose(chassis,
                    cCenter, iAmSpeed).ToPtr()}, std::pair {ReefLocation::D,
                    AlignToPose(chassis, dCenter, iAmSpeed).ToPtr()}, std::pair {ReefLocation::E, AlignToPose(chassis,
                    eCenter, iAmSpeed).ToPtr()}, std::pair {ReefLocation::F,
                    AlignToPose(chassis, fCenter, iAmSpeed).ToPtr()});
}

frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed) {
    return frc2::cmd::Select < ReefLocation
            > ([chassis, tagLayout]() {
                return findClosestReefLocation(chassis, tagLayout);
            },
            std::pair {ReefLocation::A, frc2::cmd::Sequence(AlignToPose(chassis, aTarget, true).ToPtr(),
                    AlignToPose(chassis, aRight, iAmSpeed).ToPtr())}, std::pair {ReefLocation::B, frc2::cmd::Sequence(
                    AlignToPose(chassis, bTarget, true).ToPtr(), AlignToPose(chassis, bRight, iAmSpeed).ToPtr())}, std::pair {
                    ReefLocation::C, frc2::cmd::Sequence(AlignToPose(chassis, cTarget, true).ToPtr(),
                            AlignToPose(chassis, cRight, iAmSpeed).ToPtr())}, std::pair {ReefLocation::D,
                    frc2::cmd::Sequence(AlignToPose(chassis, dTarget, true).ToPtr(),
                            AlignToPose(chassis, dRight, iAmSpeed).ToPtr())}, std::pair {ReefLocation::E,
                    frc2::cmd::Sequence(AlignToPose(chassis, eTarget, true).ToPtr(),
                            AlignToPose(chassis, eRight, iAmSpeed).ToPtr())}, std::pair {ReefLocation::F,
                    frc2::cmd::Sequence(AlignToPose(chassis, fTarget, true).ToPtr(),
                            AlignToPose(chassis, fRight, iAmSpeed).ToPtr())});
}

frc2::CommandPtr stationPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed) {
    return frc2::cmd::Select < StationLocation > ([chassis, tagLayout]() {
        return findClosestStationLocation(chassis, tagLayout);
    },
    std::pair {StationLocation::Left, AlignToPose(chassis, stationLeft, iAmSpeed).ToPtr()}, std::pair {
            StationLocation::Right, AlignToPose(chassis, stationRight, iAmSpeed).ToPtr()});
}

frc2::CommandPtr processorPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, bool iAmSpeed) {
    return AlignToPose(chassis, processor, iAmSpeed).ToPtr();
}
