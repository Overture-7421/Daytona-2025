// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignManager.h"

AlignManager::AlignManager(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout) {
	this->chassis = chassis;
	this->tagLayout = tagLayout;
}

Heading AlignManager::getHeading() {
	return this->heading;
}

void AlignManager::setHeading() {
	reefPackage = findClosestReefLocation(chassis, tagLayout);
	units::degree_t chassisHeading = chassis->getEstimatedPose().RelativeTo(reefPackage.pose).Rotation().Degrees();
	if (chassisHeading > 90_deg || chassisHeading < -90_deg) {
		this->heading = Heading::Front;
	} else {
		this->heading = Heading::Back;
	}
}

frc2::CommandPtr AlignManager::AlignToPose(ReefSide reefSide) {
	return frc2::FunctionalCommand([this, reefSide]() {
		this->reefSide = reefSide;
		
		if (this->heading == Heading::Back) {
			headingTarget = backReefOffset.headingOffset;
			this->reefOffset = backReefOffset;
		} else {
			headingTarget = frontReefOffset.headingOffset;
			this->reefOffset = frontReefOffset;
		}
		this->alignSpeedHelper = std::make_shared<AlignSpeedHelper>(chassis, tagLayout, reefSide, reefPackage, this->reefOffset, headingTarget);

		this->alignSpeedHelper->initialize();
		this->chassis->enableSpeedHelper(alignSpeedHelper.get());
	},
		[this]() {
	},
	[this](bool interrupted) {
		this->chassis->disableSpeedHelper();
		this->alignSpeedHelper = nullptr;
	},
		[this]() {
		return this->alignSpeedHelper->isAtTarget();
	},
		{ chassis }).ToPtr();
}
