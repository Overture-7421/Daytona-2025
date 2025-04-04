// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToPose.h"

AlignToPose::AlignToPose(Chassis *chassis, ReefSide reefSide, frc::AprilTagFieldLayout *tagLayout) {
    this->chassis = chassis;
    this->tagLayout = tagLayout;
    this->reefSide = reefSide;
    

    AddRequirements( {chassis});
}

void AlignToPose::Initialize() {
    ReefPackage reefPackage = findClosestReefLocation(chassis, tagLayout);
    ReefOffset reefOffset;


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

    alignSpeedHelper = std::make_shared < AlignSpeedHelper > (chassis, reefOffset, reefSide, reefPackage);


    alignSpeedHelper->initialize();
    chassis->enableSpeedHelper(alignSpeedHelper.get());
}

void AlignToPose::Execute() {
}

void AlignToPose::End(bool interrupted) {
    chassis->disableSpeedHelper();
}

bool AlignToPose::IsFinished() {
    return alignSpeedHelper->atGoal();
}
