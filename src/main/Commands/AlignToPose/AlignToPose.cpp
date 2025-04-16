// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToPose.h"

AlignToPose::AlignToPose(Chassis *chassis, ReefSide reefSide, frc::AprilTagFieldLayout *tagLayout,
        OverXboxController *driver) {
    this->chassis = chassis;
    this->driver = driver;
    this->tagLayout = tagLayout;
    this->reefSide = reefSide;

    AddRequirements( {chassis});
}

void AlignToPose::Initialize() {
    frc::SmartDashboard::PutBoolean("Aligned", false);

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

bool AlignToPose::getDriverOverride() {
    frc::Translation2d joystickPos {units::meter_t(driver->GetLeftX()), units::meter_t(driver->GetLeftY())};
    return std::abs(joystickPos.Distance( {}).value()) > 0.3;
}

bool AlignToPose::IsFinished() {
    if (alignSpeedHelper->atGoal()) {
        frc::SmartDashboard::PutBoolean("Aligned", true);
    }
    return alignSpeedHelper->atGoal() || (getDriverOverride() && alignSpeedHelper->getTargetDistance() < 0.02_m);
}
