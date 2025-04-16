// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPositions.h"
#include "Commands/AlignToPose/AlignToPose.h"

frc2::CommandPtr leftAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, OverXboxController *driver) {
    return AlignToPose(chassis, ReefSide::Left, tagLayout, driver).ToPtr();
}

frc2::CommandPtr rightAlignPos(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, OverXboxController *driver) {
    return AlignToPose(chassis, ReefSide::Right, tagLayout, driver).ToPtr();
}

