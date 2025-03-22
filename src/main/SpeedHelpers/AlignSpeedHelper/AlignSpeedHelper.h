// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

class AlignSpeedHelper: public SpeedsHelper {
public:
    AlignSpeedHelper(Chassis *chassis, const frc::AprilTagFieldLayout& layout, int targetId);
    void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
    bool atGoal();

    void initialize() override;

private:

    frc::Pose2d transformToAprilTagFrame(const frc::Pose2d& pose);

    frc::TrapezoidProfile<units::meters>::Constraints defaultConstraints {2.5_mps, 0.5_mps_sq};
    frc::TrapezoidProfile<units::meters>::Constraints minimumConstraints {0.5_mps, defaultConstraints.maxAcceleration};
    units::meter_t slowInRange = 6_m;
    units::meter_t slowDistance = 0.5_m;

    frc::ProfiledPIDController<units::meters> xPIDController {3.55, 0.0, 0.0, defaultConstraints}; //4 2.1
    frc::ProfiledPIDController<units::meters> yPIDController {3.55, 0.0, 0.0, defaultConstraints}; //4 2.1
    frc::ProfiledPIDController<units::degree> headingPIDController {5.55, 0.0, 0.0, {200_deg_per_s, 125_deg_per_s / 1_s}}; //800 500

    Chassis *chassis;
    frc::Pose2d tagPose;
};
