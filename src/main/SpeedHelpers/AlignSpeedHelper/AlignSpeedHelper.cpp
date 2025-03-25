// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignSpeedHelper.h"

#include <OvertureLib/Math/Utils.h>
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include <frc/smartdashboard/SmartDashboard.h>

double map(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

AlignSpeedHelper::AlignSpeedHelper(Chassis *chassis, frc::Pose2d targetPose, bool iAmSpeed) {
    this->chassis = chassis;
    this->targetPose = targetPose;

    this->xPIDController.SetIZone(3);
    this->xPIDController.SetTolerance(0.05_m);
    this->yPIDController.SetIZone(3);
    this->yPIDController.SetTolerance(0.05_m);
    this->headingPIDController.SetIZone(3);
    this->headingPIDController.SetTolerance(1.0_deg);
    this->headingPIDController.EnableContinuousInput(-180_deg, 180_deg);

    /*
    if (iAmSpeed) {
        this->xPIDController.SetConstraints( {3.0_mps, 1.0_mps_sq});
        this->yPIDController.SetConstraints( {3.0_mps, 1.0_mps_sq});
    }
    */
}

void AlignSpeedHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
    frc::Pose2d pose = chassis->getEstimatedPose();

    units::meter_t distanceToPose = units::math::abs(pose.Translation().Distance(flippedTargetPose.Translation()));
    frc::SmartDashboard::PutNumber("AlignTest/distanceToPose", distanceToPose.value());

    frc::TrapezoidProfile<units::meters>::Constraints actualConstraints = getConstrainst(distanceToPose);
    this->xPIDController.SetConstraints(actualConstraints);

    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);

    auto yOut = units::meters_per_second_t(yPIDController.Calculate(poseInTargetFrame.Y(), Y_TARGET));
    auto rotationOut = units::degrees_per_second_t(
            headingPIDController.Calculate(poseInTargetFrame.Rotation().Degrees(), HEADING_TARGET));


    double clampYError = std::clamp(std::abs(yPIDController.GetPositionError().value()), 0.0, 0.15);
    double yErrorToAngleMock = map(clampYError, 0.0, 0.15, 0.0, M_PI_2);
    double yScale = std::cos(yErrorToAngleMock);
    double headingScale = units::math::cos(units::math::abs(headingPIDController.GetPositionError()));
    xScale = headingScale * yScale;

    units::meter_t actualXTarget = poseInTargetFrame.X();

    if(xScale > scaleMin){
        actualXTarget = X_TARGET;
    }

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(poseInTargetFrame.X(), actualXTarget));

    if (xPIDController.AtGoal() && (xPIDController.GetGoal().position == X_TARGET)) {
        xOut = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        yOut = 0_mps;
    }

    if (headingPIDController.AtGoal()) {
        rotationOut = 0_deg_per_s;
    }

    inputSpeed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xOut * xScale * allianceMulti, yOut  * allianceMulti, rotationOut,
            chassis->getEstimatedPose().Rotation() - targetPose.Rotation());
}

frc::TrapezoidProfile<units::meters>::Constraints AlignSpeedHelper::getConstrainst(units::meter_t distance){
    if(distance > slowInRange){
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", defaultConstraints.maxVelocity.value());
        return defaultConstraints;

    } else if(distance < slowDistance){
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", minimumConstraints.maxVelocity.value());
        return minimumConstraints;

    } else {
        units::meters_per_second_t limitedVelocity = ((defaultConstraints.maxVelocity - minimumConstraints.maxVelocity)/(slowInRange - slowDistance )) * (distance - slowDistance) + minimumConstraints.maxVelocity;
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", limitedVelocity.value());
        frc::TrapezoidProfile<units::meters>::Constraints limitedConstraints {limitedVelocity, defaultConstraints.maxAcceleration};
        return limitedConstraints;
    }

}


void AlignSpeedHelper::initialize() {
    if (isRedAlliance()) {
        flippedTargetPose = pathplanner::FlippingUtil::flipFieldPose(targetPose);
        allianceMulti = -1;
    } else {
        flippedTargetPose = targetPose;
    }

    frc::Pose2d pose = chassis->getEstimatedPose();
    frc::Pose2d poseInTargetFrame = transformToTargetFrame(pose);

    xPIDController.Reset(poseInTargetFrame.X());
    yPIDController.Reset(poseInTargetFrame.Y());
    headingPIDController.Reset(poseInTargetFrame.Rotation().Radians());
}

frc::Pose2d AlignSpeedHelper::transformToTargetFrame(const frc::Pose2d& pose){
    return pose.RelativeTo(flippedTargetPose);
}

bool AlignSpeedHelper::atGoal() {
    frc::SmartDashboard::PutBoolean("AlignTest/AtGoalX", xPIDController.AtGoal() && (xPIDController.GetGoal().position == X_TARGET));
    frc::SmartDashboard::PutBoolean("AlignTest/AtGoalY", yPIDController.AtGoal());
    frc::SmartDashboard::PutBoolean("AlignTest/AtGoalH", headingPIDController.AtGoal());
    return xPIDController.AtGoal() && (xPIDController.GetGoal().position == X_TARGET) && yPIDController.AtGoal() && headingPIDController.AtGoal();
}
