// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Chassis/Chassis.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
#include <map>
#include "Enums/ReefSide.h"
#include "Commands/UtilityFunctions/ReefOffset.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"

#include <frc2/command/CommandPtr.h>
#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h"

class AlignSpeedHelper: public SpeedsHelper {
public:
    AlignSpeedHelper(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout, ReefSide reefSide, ReefPackage reefPackage,
            ReefOffset reefOffset, units::degree_t headingTarget);
    void initialize() override;
    void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;

    bool isAtTarget();

    //MOVER DERECHA es mas POSITIVOS / MOVER IZQUIERDA es mas NEGATIVOS 
    //ATRAS es mas POSITIVOS / ADELANTE es mas NEGATIVOS
    //GIRAR IZQUIERDA mas POSITIVOS / GIRAR DERECHA mas NEGATIVOS

private:
    frc::Pose2d transformToTargetFrame(const frc::Pose2d &pose);

    ReefSide reefSide;
    Chassis *chassis = nullptr;
    frc::AprilTagFieldLayout *tagLayout = nullptr;

    frc::TrapezoidProfile<units::meters>::Constraints defaultConstraints {2_mps, 0.5_mps_sq};

    frc::TrapezoidProfile<units::meters>::Constraints minimumConstraints {1_mps, 0.25_mps_sq};

    frc::ProfiledPIDController<units::meters> xPIDController {10, 0.0, 0.0, defaultConstraints}; //4 2.1

    frc::ProfiledPIDController<units::meters> yPIDController {10, 0.0, 0.0, defaultConstraints}; //4 2.1

    frc::ProfiledPIDController<units::degree> headingPIDController {5.55, 0.0, 0.0, {200_deg_per_s, 125_deg_per_s / 1_s}};

    ReefOffset reefOffset;

    units::meter_t xTarget = 0.0_m; // The target X position in the target frame
    units::meter_t yTarget = 0.0_m; // The target Y position in the target frame
    units::degree_t headingTarget = 0.0_deg; // The target heading in the target frame

    ReefPackage reefPackage;
};
