// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToPose.h"

AlignToPose::AlignToPose(Chassis *chassis, const frc::AprilTagFieldLayout& layout, int targetId) : alignSpeedHelper(chassis, layout,
        targetId) {
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
