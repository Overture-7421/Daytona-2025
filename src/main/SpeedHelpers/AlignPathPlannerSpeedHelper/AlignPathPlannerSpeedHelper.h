// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"
#include <OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h>
#include <frc/controller/ProfiledPIDController.h>
#include "Subsystems/Chassis/Chassis.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/util/FlippingUtil.h>

class AlignPathPlannerSpeedHelper: public SpeedsHelper {
public:
    AlignPathPlannerSpeedHelper(Chassis *chassis, frc::Pose2d targetPose, photon::PhotonCamera *frontLeftCamera);
    void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;
    bool atGoal();

    void initialize() override;

private:

    frc::ProfiledPIDController<units::meters> xPIDController {1, 0.0, 0.0, {1.0_mps, 0.5_mps_sq}}; //4 2.1

    frc::ProfiledPIDController<units::meters> yPIDController {2.5, 0.0, 0.0, {1.0_mps, 0.5_mps_sq}}; //4 2.1

    frc::ProfiledPIDController<units::degree> headingPIDController {2, 0.0, 0.0, {160_deg_per_s, 80_deg_per_s / 1_s}}; //800 500

    Chassis *chassis = nullptr;
    photon::PhotonCamera *frontLeftCamera = nullptr;
    frc::Pose2d targetPose;

    int id = 0;

};
