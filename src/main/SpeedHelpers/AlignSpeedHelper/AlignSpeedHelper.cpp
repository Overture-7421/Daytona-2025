// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignSpeedHelper.h"

#include <OvertureLib/Math/Utils.h>
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include <frc/smartdashboard/SmartDashboard.h>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double AlignSpeedHelper::changedXTarget = 0.0;
double AlignSpeedHelper::changedLeftTarget = 0.0;
double AlignSpeedHelper::changedRightTarget = 0.0;

AlignSpeedHelper::AlignSpeedHelper(Chassis *chassis, ReefOffset reefOffset, ReefSide direction,
        ReefPackage reefPackage) {
    this->chassis = chassis;
    this->reefPackage = reefPackage;
    this->reefOffset = reefOffset;
    this->direction = direction;

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.02_m);
    this->yPIDController.SetIZone(3);
    this->yPIDController.SetTolerance(0.02_m);
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(1.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);

}

void AlignSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    frc::Pose2d pose = chassis->getEstimatedPose();

    distanceToPose = units::math::abs(pose.Translation().Distance(reefPackage.pose.Translation()));
    frc::SmartDashboard::PutNumber("AlignTest/distanceToPose", distanceToPose.value());

    frc::TrapezoidProfile<units::meters>::Constraints actualConstraints = getConstrainst(distanceToPose);
    this->xPIDController.SetConstraints(actualConstraints);

    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);
    auto yOut = units::meters_per_second_t(yPIDController.Calculate(poseInTargetFrame.Y(), yTarget));
    auto rotationOut = units::degrees_per_second_t(
            headingPIDController.Calculate(poseInTargetFrame.Rotation().Degrees(), headingTarget));

    units::meter_t yError = units::math::abs(yPIDController.GetGoal().position - poseInTargetFrame.Y());
    units::degree_t headingError = units::math::abs(
            headingPIDController.GetGoal().position - poseInTargetFrame.Rotation().Degrees());

    double clampYError = std::clamp(yError.value(), 0.0, 0.5);
    double yErrorToAngleMock = map(clampYError, 0.0, 1.3, 0.0, M_PI_2);
    double yScale = std::cos(yErrorToAngleMock);
    double headingScale = units::math::cos(headingError);
    xScale = std::clamp((headingScale + 0.2) * yScale, 0.0, 1.0);

    units::meter_t actualXTarget = poseInTargetFrame.X();

    if (xScale > scaleMin) {
        actualXTarget = xTarget;
    }

    Logging::WriteValue("AlignOffset/XTarget", actualXTarget);

    frc::SmartDashboard::PutNumber("ReefPackage/MAth/yScale", yScale);
    frc::SmartDashboard::PutNumber("ReefPackage/MAth/headingScale", headingScale);
    frc::SmartDashboard::PutNumber("ReefPackage/XTarget/xScale", xScale);
    frc::SmartDashboard::PutNumber("ReefPackage/XTarget/actualXTarget", actualXTarget.value());

    frc::SmartDashboard::PutNumber("AlignOffset/X", AlignSpeedHelper::getModifyXTarget());
    frc::SmartDashboard::PutNumber("AlignOffset/Left", AlignSpeedHelper::getModifyLeftTarget());
    frc::SmartDashboard::PutNumber("AlignOffset/Right", AlignSpeedHelper::getModifyRightTarget());

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(poseInTargetFrame.X(), actualXTarget));

    if (xPIDController.AtGoal() && (xPIDController.GetGoal().position == xTarget) && yPIDController.AtGoal()
            && headingPIDController.AtGoal()) {
        xOut = 0_mps;
        yOut = 0_mps;
        rotationOut = 0_deg_per_s;
    }

    inputSpeed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xOut * allianceMulti, yOut * allianceMulti, rotationOut,
            chassis->getEstimatedPose().Rotation() - reefPackage.pose.Rotation());
}

frc::TrapezoidProfile<units::meters>::Constraints AlignSpeedHelper::getConstrainst(units::meter_t distance) {
    if (distance > slowInRange) {
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", defaultConstraints.maxVelocity.value());
        return defaultConstraints;

    } else if (distance < slowDistance) {
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", minimumConstraints.maxVelocity.value());
        return minimumConstraints;

    } else {
        units::meters_per_second_t limitedVelocity = ((defaultConstraints.maxVelocity - minimumConstraints.maxVelocity)
                / (slowInRange - slowDistance)) * (distance - slowDistance) + minimumConstraints.maxVelocity;
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", limitedVelocity.value());
        frc::TrapezoidProfile<units::meters>::Constraints limitedConstraints {limitedVelocity,
                defaultConstraints.maxAcceleration};
        return limitedConstraints;
    }

}

void AlignSpeedHelper::initialize() {
    if (reefPackage.alliance == frc::DriverStation::Alliance::kRed) {
        //allianceMulti = -1;
    }

    xTarget = reefOffset.xOffset + units::meter_t(changedXTarget);
    headingTarget = reefOffset.headingOffset;
    if (direction == ReefSide::Left) {
        yTarget = reefOffset.leftOffset + units::meter_t(changedLeftTarget);
    } else {
        yTarget = reefOffset.rightOffset + units::meter_t(changedRightTarget);
    }

    frc::Pose2d pose = chassis->getEstimatedPose();
    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);

    frc::ChassisSpeeds currentSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassis->getCurrentSpeeds(),
            -pose.Rotation() + reefPackage.pose.Rotation());
    xPIDController.Reset(poseInTargetFrame.X(), currentSpeeds.vx);
    yPIDController.Reset(poseInTargetFrame.Y(), currentSpeeds.vy);
    headingPIDController.Reset(poseInTargetFrame.Rotation().Radians(), currentSpeeds.omega);
}

frc::Pose2d AlignSpeedHelper::transformToTargetFrame(const frc::Pose2d &pose) {
    return pose.RelativeTo(reefPackage.pose);
}

units::meter_t AlignSpeedHelper::getTargetDistance() {
    return distanceToPose;
}

bool AlignSpeedHelper::atGoal() {
    frc::SmartDashboard::PutBoolean("AlignTest/AtGoalX",
            xPIDController.AtGoal() && (xPIDController.GetGoal().position == xTarget));
    frc::SmartDashboard::PutBoolean("AlignTest/AtGoalY", yPIDController.AtGoal());
    frc::SmartDashboard::PutBoolean("AlignTest/AtGoalH", headingPIDController.AtGoal());
    return xPIDController.AtGoal() && (xPIDController.GetGoal().position == xTarget) && yPIDController.AtGoal()
            && headingPIDController.AtGoal();
    frc::SmartDashboard::PutNumber("ReefPackage/XTarget/Direct", xTarget.value());
}

double AlignSpeedHelper::getModifyXTarget() {
    return changedXTarget;
}
void AlignSpeedHelper::setModifyXTarget(double changedTarget) {
    changedXTarget -= changedTarget;
}

double AlignSpeedHelper::getModifyLeftTarget() {
    return changedLeftTarget;
}
void AlignSpeedHelper::setModifyLeftTarget(double changedTarget) {
    changedLeftTarget -= changedTarget;
}

double AlignSpeedHelper::getModifyRightTarget() {
    return changedRightTarget;
}
void AlignSpeedHelper::setModifyRightTarget(double changedTarget) {
    changedRightTarget += changedTarget;
}

void AlignSpeedHelper::resetOffset() {
    changedXTarget = 0;
    changedLeftTarget = 0;
    changedRightTarget = 0;

}
