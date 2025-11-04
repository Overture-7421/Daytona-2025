// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignSpeedHelper.h"

AlignSpeedHelper::AlignSpeedHelper(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, ReefSide reefSide,
        ReefPackage reefPackage, ReefOffset reefOffset, units::degree_t headingTarget) {
    this->chassis = chassis;
    this->tagLayout = tagLayout;
    this->reefSide = reefSide;
    this->reefPackage = reefPackage;
    this->reefOffset = reefOffset;
    this->headingTarget = headingTarget;

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.025_m);
    this->yPIDController.SetIZone(3);
    this->yPIDController.SetTolerance(0.025_m);
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(4.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void AlignSpeedHelper::initialize() {
    if (reefSide == ReefSide::Left) {
        yTarget = reefOffset.leftOffset;
    } else if (reefSide == ReefSide::Right) {
        yTarget = reefOffset.rightOffset;
    } else {
        yTarget = reefOffset.algaeOffset;
    }

    xTarget = reefOffset.xOffset;

    frc::Pose2d pose = chassis->getEstimatedPose();
    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);

    // frc::ChassisSpeeds currentSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassis->getCurrentSpeeds(),
    //  -pose.Rotation() + reefPackage.pose.Rotation());
    xPIDController.Reset(poseInTargetFrame.X());
    yPIDController.Reset(poseInTargetFrame.Y());
    headingPIDController.Reset(poseInTargetFrame.Rotation().Degrees());

    frc::SmartDashboard::PutNumber("Align/TargetX", xTarget.value());
    frc::SmartDashboard::PutNumber("Align/TargetY", yTarget.value());
    frc::SmartDashboard::PutNumber("Align/TargetHeading", headingTarget.value());
}

void AlignSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    frc::Pose2d pose = chassis->getEstimatedPose();

    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);

    auto xSpeed = xPIDController.Calculate(poseInTargetFrame.X(), xTarget) * 1_mps;
    auto ySpeed = yPIDController.Calculate(poseInTargetFrame.Y(), yTarget) * 1_mps;
    auto headingSpeed = headingPIDController.Calculate(poseInTargetFrame.Rotation().Degrees(), headingTarget)
            * 1_deg_per_s;

    if (xPIDController.AtGoal()) {
        xSpeed = 0_mps;
    }
    if (yPIDController.AtGoal()) {
        ySpeed = 0_mps;
    }
    if (headingPIDController.AtGoal()) {
        headingSpeed = 0_deg_per_s;
    }

    inputSpeed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, units::radians_per_second_t(headingSpeed),
            pose.Rotation() - reefPackage.pose.Rotation());
}

bool AlignSpeedHelper::isAtTarget() {
    return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
}

frc::Pose2d AlignSpeedHelper::transformToTargetFrame(const frc::Pose2d &pose) {
    return pose.RelativeTo(reefPackage.pose);
}
