// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPathPlannerSpeedHelper.h"

#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include <frc/smartdashboard/SmartDashboard.h>

AlignPathPlannerSpeedHelper::AlignPathPlannerSpeedHelper(Chassis *chassis, frc::Pose2d targetPose,
        AprilTags *frontRightCamera) {
    this->chassis = chassis;
    this->frontRightCamera = frontRightCamera;
    this->targetPose = targetPose;

    this->xPIDController.SetTolerance(0.03_m);
    this->yPIDController.SetTolerance(0.03_m);
    this->headingPIDController.SetTolerance(3.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void AlignPathPlannerSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    std::optional < photon::PhotonPipelineResult > optionalResult = frontRightCamera->getCameraResult();
    if (!optionalResult.has_value()) {
        return;
    }

    photon::PhotonPipelineResult result = optionalResult.value();
    if (!result.HasTargets()) {
        return;
    }

    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();
    frc::Translation2d targetTranslation = cameraToTarget.Translation().ToTranslation2d();
    frc::Rotation2d targetRotation = cameraToTarget.Rotation().ToRotation2d();

    frc::SmartDashboard::PutNumber("AlignTarget/XTarget", targetPose.X().value());
    frc::SmartDashboard::PutNumber("AlignTarget/YTarget", targetPose.Y().value());
    frc::SmartDashboard::PutNumber("AlignTarget/RTarget", targetPose.Rotation().Degrees().value());

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(targetTranslation.X(), targetPose.X()));
    auto yOut = units::meters_per_second_t(yPIDController.Calculate(targetTranslation.Y(), targetPose.Y()));
    auto rotationOut = units::degrees_per_second_t(
            headingPIDController.Calculate(targetRotation.Degrees(), targetPose.Rotation().Degrees()));

    if (xPIDController.AtGoal()) {
        xOut = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        yOut = 0_mps;
    }

    if (headingPIDController.AtGoal()) {
        rotationOut = 0_deg_per_s;
    }

    frc::SmartDashboard::PutNumber("AlignCurrent/XCurrent", targetTranslation.X().value());
    frc::SmartDashboard::PutNumber("AlignCurrent/YCurrent", targetTranslation.Y().value());
    frc::SmartDashboard::PutNumber("AlignCurrent/RCurrent", targetRotation.Degrees().value());

    inputSpeed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(xOut, yOut, rotationOut,
            chassis->getEstimatedPose().Rotation());
}

void AlignPathPlannerSpeedHelper::initialize() {
    photon::PhotonPipelineResult result = frontRightCamera->getCameraResult().value();
    if (!result.HasTargets()) {
        return;
    }

    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();
    frc::Translation2d targetTranslation = cameraToTarget.Translation().ToTranslation2d();
    frc::Rotation2d targetRotation = cameraToTarget.Rotation().ToRotation2d();

    xPIDController.Reset(targetTranslation.X());
    yPIDController.Reset(targetTranslation.Y());
    headingPIDController.Reset(targetRotation.Degrees());
}

bool AlignPathPlannerSpeedHelper::atGoal() {
    return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
}
