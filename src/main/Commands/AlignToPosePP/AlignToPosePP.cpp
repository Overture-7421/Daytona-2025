// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToPosePP.h"

AlignToPosePP::AlignToPosePP(Chassis *chassis, frc::Pose2d pose2d, AprilTags *frontRightCamera) : alignSpeedHelper(
        chassis, pose2d, frontRightCamera) {
    this->chassis = chassis;

    AddRequirements( {chassis});
}

void AlignToPosePP::Initialize() {
    alignSpeedHelper.initialize();
    chassis->enableSpeedHelper(&alignSpeedHelper);
}

void AlignToPosePP::Execute() {
}

void AlignToPosePP::End(bool interrupted) {
    chassis->disableSpeedHelper();
}

bool AlignToPosePP::IsFinished() {
    return alignSpeedHelper.atGoal();
}
