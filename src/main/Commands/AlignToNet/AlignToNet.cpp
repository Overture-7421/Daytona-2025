// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToNet.h"

AlignToNet::AlignToNet(Chassis *chassis, frc::Pose2d pose2d) : alignNetSpeedHelper(chassis, pose2d) {
    this->chassis = chassis;

    AddRequirements( {});
}

void AlignToNet::Initialize() {
    alignNetSpeedHelper.initialize();
    chassis->enableSpeedHelper(&alignNetSpeedHelper);
}

void AlignToNet::Execute() {
}

void AlignToNet::End(bool interrupted) {
    chassis->disableSpeedHelper();
}

bool AlignToNet::IsFinished() {
    return false;
}
