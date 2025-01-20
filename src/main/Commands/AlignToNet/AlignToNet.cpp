// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignToNet.h"

AlignToNet::AlignToNet(Chassis *chassis, frc::Pose2d pose2d) : alignNet(chassis, pose2d) {
    this->chassis = chassis;
    this->pose2d = pose2d;

    AddRequirements( {chassis});
}

void AlignToNet::Initialize() {
    chassis->enableSpeedHelper(&alignNet);
}

void AlignToNet::Execute() {
}

void AlignToNet::End(bool interrupted) {
    chassis->disableSpeedHelper();
}

bool AlignToNet::IsFinished() {
    return alignNet.atGoal();
}
