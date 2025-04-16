// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "frc/apriltag/AprilTagFieldLayout.h"
#include <frc/DriverStation.h>
#include "Enums/ReefLocation.h"

struct ReefPackage {

    frc::DriverStation::Alliance alliance;
    ReefLocation reefLocation;
    frc::Pose2d pose;

};
