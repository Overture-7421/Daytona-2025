// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

class AlignSpeedHelper: public SpeedsHelper {
public:
    AlignSpeedHelper(Chassis *chassis, frc::Pose2d targetPose, bool iAmSpeed);
    void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
    bool atGoal();

    void initialize() override;

private:

    frc::TrapezoidProfile<units::meters>::Constraints getConstrainst(units::meter_t distance);
    frc::Pose2d transformToTargetFrame(const frc::Pose2d& pose);

    frc::TrapezoidProfile<units::meters>::Constraints defaultConstraints {2.5_mps, 1.0_mps_sq};

    frc::TrapezoidProfile<units::meters>::Constraints minimumConstraints {0.5_mps, defaultConstraints.maxAcceleration};

    frc::ProfiledPIDController<units::meters> xPIDController {5.55, 0.0, 0.0, defaultConstraints}; //4 2.1

    frc::ProfiledPIDController<units::meters> yPIDController {5.55, 0.0, 0.0, defaultConstraints}; //4 2.1

    frc::ProfiledPIDController<units::degree> headingPIDController {5.55, 0.0, 0.0, {200_deg_per_s, 125_deg_per_s / 1_s}}; //800 500

    Chassis *chassis;
    frc::Pose2d targetPose;
    frc::Pose2d flippedTargetPose;

    units::meter_t slowInRange = 6_m;
    units::meter_t slowDistance = 0.5_m;

};
