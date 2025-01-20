// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Align.h"

#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/pathfinding/Pathfinder.h>
#include <pathplanner/lib/util/FlippingUtil.h>
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>

Align::Align(Chassis *chassis, frc::Pose2d targetPose) {
    this->chassis = chassis;
    this->targetPose = targetPose;

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.15_m);
    this->yPIDController.SetIZone(3);
    this->yPIDController.SetTolerance(0.15_m);
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(3.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void Align::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    frc::Pose2d pose = chassis->getEstimatedPose();

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(pose.X(), targetPose.X()));
    auto yOut = units::meters_per_second_t(yPIDController.Calculate(pose.Y(), targetPose.Y()));
    auto rotationOut = units::degrees_per_second_t(
            headingPIDController.Calculate(pose.Rotation().Degrees(), targetPose.Rotation().Degrees()));

    if (xPIDController.AtGoal()) {
        xOut = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        yOut = 0_mps;
    }

    if (headingPIDController.AtGoal()) {
        rotationOut = 0_deg_per_s;
    }

    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xOut, yOut, rotationOut,
            chassis->getEstimatedPose().Rotation());
    speeds = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);

    inputSpeed = speeds;
}

void Align::initialize() {
    xPIDController.Reset(chassis->getEstimatedPose().X());
    yPIDController.Reset(chassis->getEstimatedPose().Y());
    headingPIDController.Reset(chassis->getEstimatedPose().Rotation().Radians());

    if (isRedAlliance()) {
        targetPose = pathplanner::FlippingUtil::flipFieldPose(targetPose);
    } else {
        targetPose = targetPose;
    }
}

bool Align::atGoal() {
    return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
}
