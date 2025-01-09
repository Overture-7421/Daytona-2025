// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveCommand.h"
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>
#include <cmath>
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>

DriveCommand::DriveCommand(Chassis* chassis, OverXboxController* gamepad) : headingSpeedsHelper{ headingController, chassis } {
	this->chassis = chassis;
	this->gamepad = gamepad;
	// Use addRequirements() here to declare subsystem dependencies.
	AddRequirements({ chassis });
}

// Called when the command is initially scheduled.
void DriveCommand::Initialize() {
	if (isRedAlliance()) {
		allianceMulti = -1;
	} else {
		allianceMulti = 1;
	}

}

// Called repeatedly when this Command is scheduled to run
void DriveCommand::Execute() {
	frc::Rotation2d targetAngle{ gamepad->getRightStickDirection() };

	if (allianceMulti == -1) {
		targetAngle = targetAngle.RotateBy({ 180_deg });
	}

	double squares = sqrt(gamepad->GetRightY() * gamepad->GetRightY() + gamepad->GetRightX() * gamepad->GetRightX());

	if (squares > 0.71) {
		if (speedHelperMoved == false) {
			speedHelperMoved = true;
			chassis->enableSpeedHelper(&headingSpeedsHelper);
		}

	} else if (speedHelperMoved == true) {
		speedHelperMoved = false;
		chassis->disableSpeedHelper();
	}

	headingSpeedsHelper.setTargetAngle(targetAngle);

	auto xSpeed = xInput.Calculate(Utils::ApplyAxisFilter(allianceMulti * -gamepad->GetHID().GetRawAxis(1), 0.2, 0.5) * chassis->getMaxModuleSpeed());
	auto ySpeed = yInput.Calculate(Utils::ApplyAxisFilter(allianceMulti * -gamepad->GetHID().GetRawAxis(0), 0.2, 0.5) * chassis->getMaxModuleSpeed());
	auto rotationSpeed = (gamepad->getTwist() * 5.0_tps);

	frc::ChassisSpeeds speeds = frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, chassis->getEstimatedPose().Rotation()), 0.02_s);
	chassis->setTargetSpeeds(speeds);

}

// Called once the command ends or is interrupted.
void DriveCommand::End(bool interrupted) {
	chassis->disableSpeedHelper();

}

// Returns true when the command should end.
bool DriveCommand::IsFinished() {
	return false;
}
