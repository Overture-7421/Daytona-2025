// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Chassis/Chassis.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include <map>
#include "Enums/ReefSide.h"
#include "Commands/UtilityFunctions/ReefOffset.h"
#include "Enums/Heading.h"

#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

class AlignManager {
public:
    AlignManager(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout);

    void getReefOffset(ReefSide reefSide);

    frc2::CommandPtr AlignToPose(ReefSide reefSide);

    frc2::CommandPtr setHeading(Heading heading);
    Heading getHeading();

    //MOVER DERECHA es mas POSITIVOS / MOVER IZQUIERDA es mas NEGATIVOS 
    //ATRAS es mas POSITIVOS / ADELANTE es mas NEGATIVOS
    //GIRAR IZQUIERDA mas POSITIVOS / GIRAR DERECHA mas NEGATIVOS

    //IZQUIERDA, DERECHA, ADELANTE/ATRAS, ROTACION, ALGA
    const ReefOffset defaultReefOffset {0.139_m, 0.476_m, 0.59_m, 180.0_deg, 0.0_m};

    std::map<ReefLocation, ReefOffset> alignPositionsMap;
    const std::map<ReefLocation, ReefOffset> alignInRed = {};
    const std::map<ReefLocation, ReefOffset> alignInBlue = {};

private:
    Chassis *chassis;
    frc::AprilTagFieldLayout *tagLayout;

    ReefOffset reefOffset;

    units::meter_t xTarget = 0.0_m; // The target X position in the target frame
    units::meter_t yTarget = 0.0_m; // The target Y position in the target frame
    units::degree_t headingTarget = 0.0_deg; // The target heading in the target frame

    frc::Pose2d targetPose;

    Heading heading = Heading::Front;
};
