// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignSpeedHelper.h"

#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>
#include <frc/smartdashboard/SmartDashboard.h>

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



    if(distanceToPose > slowInRange){
        this->xPIDController.SetConstraints(defaultConstraints);
        this->yPIDController.SetConstraints(defaultConstraints);
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", defaultConstraints.maxVelocity.value());

    } else if(distanceToPose < slowDistance){
        this->xPIDController.SetConstraints(minimumConstraints);
        this->yPIDController.SetConstraints(minimumConstraints);

        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", minimumConstraints.maxVelocity.value());

    } else {
        units::meters_per_second_t limitedVelocity = ((defaultConstraints.maxVelocity - minimumConstraints.maxVelocity)/(slowInRange - slowDistance )) * (distanceToPose - slowDistance) + minimumConstraints.maxVelocity;
        frc::SmartDashboard::PutNumber("AlignTest/limitedVelocity", limitedVelocity.value());
        frc::TrapezoidProfile<units::meters>::Constraints limitedConstraints {limitedVelocity, defaultConstraints.maxAcceleration};

        this->xPIDController.SetConstraints(limitedConstraints);
        this->yPIDController.SetConstraints(limitedConstraints);
    }


    frc::SmartDashboard::PutNumber("AlignTarget/XTarget", flippedTargetPose.X().value());
    frc::SmartDashboard::PutNumber("AlignTarget/YTarget", flippedTargetPose.Y().value());
    frc::SmartDashboard::PutNumber("AlignTarget/RTarget", flippedTargetPose.Rotation().Degrees().value());

    auto xOut = units::meters_per_second_t(xPIDController.Calculate(pose.X(), flippedTargetPose.X()));
    auto yOut = units::meters_per_second_t(yPIDController.Calculate(pose.Y(), flippedTargetPose.Y()));
    auto rotationOut = units::degrees_per_second_t(
            headingPIDController.Calculate(pose.Rotation().Degrees(), flippedTargetPose.Rotation().Degrees()));

    if (xPIDController.AtGoal()) {
        xOut = 0_mps;
    }

    if (yPIDController.AtGoal()) {
        yOut = 0_mps;
    }

    if (headingPIDController.AtGoal()) {
        rotationOut = 0_deg_per_s;
    }

    frc::SmartDashboard::PutNumber("AlignCurrent/XCurrent", pose.X().value());
    frc::SmartDashboard::PutNumber("AlignCurrent/YCurrent", pose.Y().value());
    frc::SmartDashboard::PutNumber("AlignCurrent/RCurrent", pose.Rotation().Degrees().value());

    inputSpeed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xOut, yOut, rotationOut,
            chassis->getEstimatedPose().Rotation());
}

void AlignSpeedHelper::initialize() {
    if (isRedAlliance()) {
        flippedTargetPose = pathplanner::FlippingUtil::flipFieldPose(targetPose);
    } else {
        flippedTargetPose = targetPose;
    }

    xPIDController.Reset(chassis->getEstimatedPose().X());
    yPIDController.Reset(chassis->getEstimatedPose().Y());
    headingPIDController.Reset(chassis->getEstimatedPose().Rotation().Radians());
}

bool AlignSpeedHelper::atGoal() {
    return xPIDController.AtGoal() && yPIDController.AtGoal() && headingPIDController.AtGoal();
}
