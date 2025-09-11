// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignManager.h"

AlignManager::AlignManager(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    this->chassis = chassis;
    this->tagLayout = tagLayout;
}

void AlignManager::getReefOffset(ReefSide reefSide) {
    ReefPackage reefPackage = findClosestReefLocation(chassis, tagLayout);
    if (reefPackage.alliance == frc::DriverStation::Alliance::kRed) {
        alignPositionsMap = alignInRed;
    } else {
        alignPositionsMap = alignInBlue;
    }

    if (alignPositionsMap.contains(reefPackage.reefLocation)) {
        reefOffset = alignPositionsMap.at(reefPackage.reefLocation);
    } else {
        reefOffset = defaultReefOffset;
    }

    if (reefSide == ReefSide::Left) {
        yTarget = reefOffset.leftOffset;
    } else if (reefSide == ReefSide::Right) {
        yTarget = reefOffset.rightOffset;
    } else {
        yTarget = reefOffset.algaeOffset;
    }

    if (reefPackage.algaePose == AlgaePose::Up) {
        setAlgaePose(AlgaePose::Up);
    } else if (reefPackage.algaePose == AlgaePose::Down) {
        setAlgaePose(AlgaePose::Down);
    }

    units::degree_t chassisHeading = chassis->getEstimatedPose().RelativeTo(reefPackage.pose).Rotation().Degrees();
    if (chassisHeading < 90_deg || chassisHeading > -90_deg) {
        headingTarget = reefOffset.headingOffset;
        setHeading(Heading::Front);
    } else {
        headingTarget = reefOffset.headingOffset + 180_deg;
        setHeading(Heading::Back);
    }

    targetPose = reefPackage.pose.TransformBy( {reefOffset.xOffset, yTarget, headingTarget});
}

pathplanner::PathConstraints constraints = pathplanner::PathConstraints(4.0_mps, 2.5_mps_sq, 540_deg_per_s,
	720_deg_per_s_sq);

frc2::CommandPtr AlignManager::AlignToPose(ReefSide reefSide) {
    return frc2::cmd::Sequence(frc2::cmd::RunOnce([this, reefSide] {
        getReefOffset(reefSide);
    }),
    frc2::cmd::Defer([this]() {
        return pathplanner::AutoBuilder::pathfindToPose(targetPose, constraints, 0_mps);
    }, {chassis})  // Add chassis as requirement
            );
}

void AlignManager::setHeading(Heading heading) {
    this->heading = heading;
}

Heading AlignManager::getHeading() {
    return heading;
}

void AlignManager::setAlgaePose(AlgaePose algaePose) {
    this->algaePose = algaePose;
}

AlgaePose AlignManager::getAlgaePose() {
    return algaePose;
}
