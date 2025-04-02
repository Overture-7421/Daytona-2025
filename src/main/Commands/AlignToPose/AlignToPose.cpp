// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToPose.h"

AlignToPose::AlignToPose(Chassis *chassis, ReefSide direction, ReefPackage reefPackage) {
    this->chassis = chassis;

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

    alignSpeedHelper = std::make_shared < AlignSpeedHelper > (chassis, reefOffset, direction, reefPackage);

    AddRequirements( {chassis});
}

void AlignToPose::Initialize() {
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
