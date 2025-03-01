// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPathPlannerSpeedHelper.h"

#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include <frc/smartdashboard/SmartDashboard.h>

AlignPathPlannerSpeedHelper::AlignPathPlannerSpeedHelper(Chassis *chassis, frc::Pose2d targetPose,
        photon::PhotonCamera *frontLeftCamera) {
    this->chassis = chassis;
    this->frontLeftCamera = frontLeftCamera;
    this->targetPose = targetPose;

    this->xPIDController.SetTolerance(0.03_m);
    this->yPIDController.SetTolerance(0.01_m);
    this->headingPIDController.SetTolerance(3.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void AlignPathPlannerSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    std::vector < photon::PhotonPipelineResult > vectorResult = frontLeftCamera->GetAllUnreadResults();

    if (vectorResult.empty()) {
        return;
    }

    photon::PhotonPipelineResult result = vectorResult[0];

    if (result.GetBestTarget().fiducialId != id) {
        return;
    }

    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();
    frc::Translation2d targetTranslation = cameraToTarget.Translation().ToTranslation2d();

    frc::SmartDashboard::PutNumber("AlignTarget/XTarget", targetPose.X().value());
    frc::SmartDashboard::PutNumber("AlignTarget/YTarget", targetPose.Y().value());

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(targetTranslation.X(), targetPose.X()));
    auto yOut = units::meters_per_second_t(yPIDController.Calculate(targetTranslation.Y(), targetPose.Y()));

    if (xPIDController.AtGoal()) {
        xOut = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        yOut = 0_mps;
    }

    frc::SmartDashboard::PutNumber("AlignCurrent/XCurrent", targetTranslation.X().value());
    frc::SmartDashboard::PutNumber("AlignCurrent/YCurrent", targetTranslation.Y().value());

    // inputSpeed.vx = xOut;
    // inputSpeed.vy = yOut;
    // inputSpeed.omega = rotationOut;

    inputSpeed.vy = -yOut;
    inputSpeed.vx = -xOut;

}

void AlignPathPlannerSpeedHelper::initialize() {
    std::vector < photon::PhotonPipelineResult > vectorResult = frontLeftCamera->GetAllUnreadResults();

    if (vectorResult.empty()) {
        return;
    }

    photon::PhotonPipelineResult result = vectorResult[0];

    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();
    frc::Translation2d targetTranslation = cameraToTarget.Translation().ToTranslation2d();
    id = result.GetBestTarget().fiducialId;
    xPIDController.Reset(targetTranslation.X());
    yPIDController.Reset(targetTranslation.Y());
}

bool AlignPathPlannerSpeedHelper::atGoal() {
    return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
}
