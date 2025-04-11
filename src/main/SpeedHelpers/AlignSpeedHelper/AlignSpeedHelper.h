// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include "Enums/ReefSide.h"
#include "Commands/UtilityFunctions/ReefOffset.h"

class AlignSpeedHelper: public SpeedsHelper {
public:
    AlignSpeedHelper(Chassis *chassis, ReefOffset reefOffset, ReefSide direction, ReefPackage reefPackage);
    void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
    bool atGoal();

    void initialize() override;

private:

    frc::TrapezoidProfile<units::meters>::Constraints getConstrainst(units::meter_t distance);
    frc::Pose2d transformToTargetFrame(const frc::Pose2d &pose);

    frc::TrapezoidProfile<units::meters>::Constraints defaultConstraints {3_mps, 1_mps_sq};

    frc::TrapezoidProfile<units::meters>::Constraints minimumConstraints {1_mps, 0.25_mps_sq};

    frc::ProfiledPIDController<units::meters> xPIDController {5.55, 0.0, 0.0, defaultConstraints}; //4 2.1

    frc::ProfiledPIDController<units::meters> yPIDController {5.55, 0.0, 0.0, defaultConstraints}; //4 2.1

    frc::ProfiledPIDController<units::degree> headingPIDController {5.55, 0.0, 0.0, {200_deg_per_s, 125_deg_per_s / 1_s}}; //800 500

    Chassis *chassis;

    ReefPackage reefPackage;
    ReefOffset reefOffset;
    ReefSide direction;

    units::meter_t slowInRange = 6_m;
    units::meter_t slowDistance = 0.75_m;
    double allianceMulti = 1.0;

    units::meter_t xTarget = 0.0_m; // The target X position in the target frame
    units::meter_t yTarget = 0.0_m; // The target Y position in the target frame
    units::degree_t headingTarget = 0.0_deg; // The target heading in the target frame

    // xScale changes depending on how close heading and Y are to the target. We only move in X (forward) after Y and heading are close to the target.
    const double scaleMin = 0.7; // At which xScale we start to track xTarget
    double xScale = 0.0;

};
