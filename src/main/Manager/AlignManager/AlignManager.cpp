// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignManager.h"

AlignManager::AlignManager(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    this->chassis = chassis;
    this->tagLayout = tagLayout;

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.01_m);
    this->yPIDController.SetIZone(3);
    this->yPIDController.SetTolerance(0.01_m);
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(0.5_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void AlignManager::initialize() {
	reefPackage = findClosestReefLocation(chassis, tagLayout);
	if (reefPackage.alliance == frc::DriverStation::Alliance::kRed) {
		alignPositionsMap = alignInRed;
	} else {
		alignPositionsMap = alignInBlue;
	}

    if (alignPositionsMap.contains(reefPackage.reefLocation)) {
        reefOffset = alignPositionsMap.at(reefPackage.reefLocation);
    } else {
        reefOffset = defaultReefOffset;
    }

    if (reefSide == ReefSide::Left) {
        yTarget = reefOffset.leftOffset;
    } else if (reefSide == ReefSide::Right) {
        yTarget = reefOffset.rightOffset;
    } else {
        yTarget = reefOffset.algaeOffset;
    }

    xTarget = reefOffset.xOffset;

    if (reefPackage.algaePose == AlgaePose::Up) {
        setAlgaePose(AlgaePose::Up);
    } else if (reefPackage.algaePose == AlgaePose::Down) {
        setAlgaePose(AlgaePose::Down);
    }

    units::degree_t chassisHeading = chassis->getEstimatedPose().RelativeTo(reefPackage.pose).Rotation().Degrees();
    if (chassisHeading < 90_deg || chassisHeading > -90_deg) {
        headingTarget = reefOffset.headingOffset;
        setHeading(Heading::Front);
    } else {
        headingTarget = reefOffset.headingOffset + 180_deg;
        setHeading(Heading::Back);
    }

    frc::Pose2d pose = chassis->getEstimatedPose();
    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);

    frc::ChassisSpeeds currentSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassis->getCurrentSpeeds(),
            -pose.Rotation() + reefPackage.pose.Rotation());
    xPIDController.Reset(poseInTargetFrame.X(), currentSpeeds.vx);
    yPIDController.Reset(poseInTargetFrame.Y(), currentSpeeds.vy);
    headingPIDController.Reset(poseInTargetFrame.Rotation().Degrees(), currentSpeeds.omega);

    frc::SmartDashboard::PutNumber("Align/TargetX", xTarget.value());
    frc::SmartDashboard::PutNumber("Align/TargetY", yTarget.value());
    frc::SmartDashboard::PutNumber("Align/TargetHeading", headingTarget.value());
}

void AlignManager::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
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

	inputSpeed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed,
		units::radians_per_second_t(headingSpeed), pose.Rotation() - reefPackage.pose.Rotation());
}

frc::Pose2d AlignManager::transformToTargetFrame(const frc::Pose2d &pose) {
    return pose.RelativeTo(reefPackage.pose);
}

frc2::CommandPtr AlignManager::AlignToPose(ReefSide reefSide) {
    return frc2::FunctionalCommand([this, reefSide]() {
        this->reefSide = reefSide;
        this->initialize();
        this->chassis->enableSpeedHelper(this);
    },
    [this]() {
    },
    [this](bool interrupted) {
        this->chassis->disableSpeedHelper();
    },
    [this]() {
        return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
    },
    {chassis}).ToPtr();
}

void AlignManager::setHeading(Heading heading) {
    this->heading = heading;
}

Heading AlignManager::getHeading() {
    return heading;
}

void AlignManager::setAlgaePose(AlgaePose algaePose) {
    this->algaePose = algaePose;
}

AlgaePose AlignManager::getAlgaePose() {
    return algaePose;
}
