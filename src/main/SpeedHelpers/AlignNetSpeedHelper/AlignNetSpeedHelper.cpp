// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignNetSpeedHelper.h"

#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/pathfinding/Pathfinder.h>
#include <pathplanner/lib/util/FlippingUtil.h>
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>

AlignNetSpeedHelper::AlignNetSpeedHelper(Chassis *chassis, frc::Pose2d targetPose) {
    this->chassis = chassis;
    this->targetPose = targetPose;

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.15_m);
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(3.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void AlignNetSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    frc::Pose2d pose = chassis->getEstimatedPose();

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(pose.X(), targetPose.X()));
    auto yOut = units::meters_per_second_t(inputSpeed.vy());
    auto rotationOut = units::degrees_per_second_t(
            headingPIDController.Calculate(pose.Rotation().Degrees(), targetPose.Rotation().Degrees()));

    if (xPIDController.AtGoal()) {
        xOut = 0_mps;
    }

    if (headingPIDController.AtGoal()) {
        rotationOut = 0_deg_per_s;
    }

    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xOut, yOut, rotationOut,
            chassis->getEstimatedPose().Rotation());
    speeds = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);

    inputSpeed = speeds;
}

void AlignNetSpeedHelper::initialize() {
    xPIDController.Reset(chassis->getEstimatedPose().X());
    headingPIDController.Reset(chassis->getEstimatedPose().Rotation().Radians());

    if (isRedAlliance()) {
        targetPose = pathplanner::FlippingUtil::flipFieldPose(targetPose);
    } else {
        targetPose = targetPose;
    }
}

bool AlignNetSpeedHelper::atGoal() {
    return xPIDController.AtGoal() && headingPIDController.AtGoal();
}
