// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignSpeedHelper.h"

#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include <frc/smartdashboard/SmartDashboard.h>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

AlignSpeedHelper::AlignSpeedHelper(Chassis *chassis, const frc::AprilTagFieldLayout& layout, int targetId) {
    this->chassis = chassis;
    this->tagPose = layout.GetTagPose(targetId).value().ToPose2d();

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.05_m);
    this->yPIDController.SetIZone(3);
    this->yPIDController.SetTolerance(0.05_m);
    
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(1.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);
}

void AlignSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    frc::Pose2d pose = chassis->getEstimatedPose();

    frc::Pose2d poseInTagFrame = transformToAprilTagFrame(pose);

    auto outX = units::meters_per_second_t(xPIDController.Calculate(poseInTagFrame.X(), 0.25_m)); // Aim .25m in front of april tag
    auto outY = units::meters_per_second_t(yPIDController.Calculate(poseInTagFrame.Y(), 0.0_m)); // Be perfectly aligned with april tag on Y
    auto outHeading = units::degrees_per_second_t(headingPIDController.Calculate(poseInTagFrame.Rotation().Degrees(), 180.0_deg)); // Face toward the april tag
    
    // Anything above 0.25m of error in Y turns to M_PI_2, which using cos scale prevents X movement.
    double yErrorToAngleMock = map(std::abs(yPIDController.GetPositionError().value()), 0.0, 0.25, 0.0, M_PI_2);
    double yScale = std::cos(std::clamp(yErrorToAngleMock, 0.0, M_PI_2)); 
    double headingScale = units::math::cos(headingPIDController.GetPositionError()); 
    double xScale = headingScale * yScale;

    if (xPIDController.AtGoal()) {
        outX = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        outY = 0_mps;
    }

    if (headingPIDController.AtGoal()) {
        outHeading = 0_deg_per_s;
    }
    inputSpeed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(outX * xScale, outY, outHeading, chassis->getEstimatedPose().Rotation() - tagPose.Rotation());

    frc::SmartDashboard::PutNumber("AlignSpeedHelper/X", poseInTagFrame.X().value());
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/Y", poseInTagFrame.Y().value());
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/Heading", poseInTagFrame.Rotation().Degrees().value());


    frc::SmartDashboard::PutNumber("AlignSpeedHelper/OutX", outX.value());
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/OutY", outY.value());
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/OutOmega", outHeading.value());
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/yScale", yScale);
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/headingScale", headingScale);
    frc::SmartDashboard::PutNumber("AlignSpeedHelper/xScale", xScale);

}

void AlignSpeedHelper::initialize() {
    frc::Pose2d poseInTagFrame = transformToAprilTagFrame(chassis->getEstimatedPose());

    xPIDController.Reset(poseInTagFrame.X());
    yPIDController.Reset(poseInTagFrame.Y());
    headingPIDController.Reset(poseInTagFrame.Rotation().Degrees());
}

frc::Pose2d AlignSpeedHelper::transformToAprilTagFrame(const frc::Pose2d& pose) {
    return pose.RelativeTo(tagPose);
}

bool AlignSpeedHelper::atGoal() {
    return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
}
