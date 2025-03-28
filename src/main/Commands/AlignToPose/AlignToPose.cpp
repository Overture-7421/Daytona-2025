// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToPose.h"

AlignToPose::AlignToPose(Chassis *chassis, frc::Pose2d pose2d, bool iAmSpeed) : alignSpeedHelper(chassis, pose2d,
        iAmSpeed) {
    this->chassis = chassis;

    AddRequirements( {chassis});
}

void AlignToPose::Initialize() {
    alignSpeedHelper.initialize();
    chassis->enableSpeedHelper(&alignSpeedHelper);
}

void AlignToPose::Execute() {
}

void AlignToPose::End(bool interrupted) {
    chassis->disableSpeedHelper();
}

bool AlignToPose::IsFinished() {
    return alignSpeedHelper.atGoal();
}
