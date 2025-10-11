// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/Chassis/Chassis.h"
#include "Enums/ReefSide.h"
#include "Enums/Heading.h"
#include "Commands/UtilityFunctions/ReefOffset.h"
#include "SpeedHelpers/AlignSpeedHelper/AlignSpeedHelper.h"
#include "Commands/UtilityFunctions/UtilityFunctions.h"

#include <frc2/command/CommandPtr.h>

class AlignManager {
public:
	AlignManager(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout);
	void initialize();
	Heading getHeading();
	void setHeading();
	void resetSpeedHelper();
	frc2::CommandPtr AlignToPose(ReefSide reefSide);

private:
	//IZQUIERDA, DERECHA, ADELANTE/ATRAS, ROTACION, ALGA
	const ReefOffset frontReefOffset{ -0.320_m, 0.020_m, 0.48_m, 180.0_deg, -0.178_m };
	const ReefOffset backReefOffset{ 0.0_m, 0.38_m, 0.48_m, 0.0_deg, 0.0_m };

	std::shared_ptr<AlignSpeedHelper> alignSpeedHelper;
	ReefSide reefSide;
	ReefOffset reefOffset;
	Chassis* chassis = nullptr;
	frc::AprilTagFieldLayout* tagLayout = nullptr;
	Heading heading = Heading::Front;
	ReefPackage reefPackage;
	units::degree_t headingTarget;
};
