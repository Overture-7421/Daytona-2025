// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {

	ConfigureBindings();
	chassis.setAcceptingVisionMeasurements(true);
	frc::DriverStation::SilenceJoystickConnectionWarning(true);

	pathplanner::NamedCommands::registerCommand("FirstL4Front", std::move(L4CommandAutoFront(&stateManager)));

	pathplanner::NamedCommands::registerCommand("FirstL4Back", std::move(L4CommandAutoBack(&stateManager)));

	pathplanner::NamedCommands::registerCommand("LeftAlign",
		std::move(leftAlignPos(&alignManager)).BeforeStarting(frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})));

	pathplanner::NamedCommands::registerCommand("RightAlign",
		std::move(rightAlignPos(&alignManager)).BeforeStarting(frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})));

	pathplanner::NamedCommands::registerCommand("L4", std::move(L4Command(&stateManager, &alignManager)));

	pathplanner::NamedCommands::registerCommand("Sustained", std::move(SustainedCommands(&stateManager)));

	pathplanner::NamedCommands::registerCommand("CoralHold", std::move(stateManager.L1PositionToCoralHoldAuto()));

	pathplanner::NamedCommands::registerCommand("Confirm", std::move(ConfirmCommand(&stateManager)));

	pathplanner::NamedCommands::registerCommand("Intake", std::move(stateManager.SustainedToIntake()));

	pathplanner::NamedCommands::registerCommand("AlgaeHold", std::move(AlgaeHoldCommand(&stateManager)));

	pathplanner::NamedCommands::registerCommand("L1", std::move(L1Command(&stateManager)));

	autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
	frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

}

void RobotContainer::ConfigureBindings() {
	ConfigDriverBindings();
	ConfigOperatorBindings();
	ConfigMixedBindigs();
	ConfigDefaultCommands();
	ConfigCharacterizationBindings();
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	chassis.setAcceptingVisionMeasurements(true);

	return autoChooser.GetSelected();
}

void RobotContainer::ConfigDriverBindings() {
	chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());
	driver.Back().OnTrue(ResetHeading(&chassis));

	driver.LeftTrigger().WhileTrue(stateManager.SustainedToIntake().Unless([this] {
		return grabber.isAlgaeIn();
	}));
	driver.LeftTrigger().OnFalse(L1Command(&stateManager).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	driver.RightTrigger().WhileTrue(stateManager.L1PositionIntake());
	driver.RightTrigger().OnFalse(stateManager.L1PositionClosed());

	driver.POVUp().OnTrue(PassCommand(&stateManager).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	driver.RightBumper().Debounce(200_ms).WhileTrue(ConfirmCommand(&stateManager));
	driver.RightBumper().Debounce(200_ms).OnFalse(SustainedConfirmedCommands(&stateManager));

	driver.LeftBumper().Debounce(200_ms).WhileTrue(AlgaeGroundCommand(&stateManager));
	driver.LeftBumper().Debounce(200_ms).OnFalse(
		frc2::cmd::Either(AlgaeHoldCommand(&stateManager), SustainedCommands(&stateManager), [this] {
		return grabber.isAlgaeIn();
	}));

	driver.POVRight().WhileTrue(NetCommand(&stateManager));

	driver.POVDown().OnFalse(frc2::cmd::RunOnce([this] {
		chassis.disableSpeedHelper();
	}, { &chassis }));

	toInitial.OnTrue(stateManager.AllToInitial().AndThen(frc2::cmd::RunOnce([this] {
		frc::SmartDashboard::PutBoolean("To-Initial", false);
	})));

	driver.A().WhileTrue(grabber.setCharacterization(GrabberConstants::GrabberSpitAlgae));
	driver.A().OnFalse(grabber.setCharacterization(GrabberConstants::GrabberRollersZero));

	driver.Y().WhileTrue(algaeAlignPos(&alignManager));
}

void RobotContainer::ConfigOperatorBindings() {

	oprtr.LeftBumper().WhileTrue(stateManager.AlgaeHoldToProcessor());
	oprtr.LeftBumper().OnFalse(SustainedCommands(&stateManager));

	oprtr.RightBumper().WhileTrue(stateManager.AlgaeHoldToNet());
	oprtr.RightBumper().OnFalse(SustainedCommands(&stateManager));

	oprtr.A().WhileTrue(L1Command(&stateManager));
	oprtr.A().OnFalse(SustainedCommands(&stateManager));

	oprtr.B().WhileTrue(stateManager.CoralHoldToL2Front());
	oprtr.B().OnFalse(SustainedCommands(&stateManager));

	oprtr.X().WhileTrue(stateManager.CoralHoldToL3Front());
	oprtr.X().OnFalse(SustainedCommands(&stateManager));

	oprtr.Y().WhileTrue(stateManager.CoralHoldToL4Front());
	oprtr.Y().OnFalse(SustainedCommands(&stateManager));

	oprtr.POVUp().WhileTrue(AlgaeHighManualCommand(&stateManager));
	oprtr.POVUp().OnFalse(frc2::cmd::Either(AlgaeHoldCommand(&stateManager), SustainedCommands(&stateManager), [this] {
		return grabber.isAlgaeIn();
	}));

	oprtr.POVDown().WhileTrue(AlgaeLowManualCommand(&stateManager));
	// oprtr.POVDown().OnFalse(
	// 	frc2::cmd::Either(AlgaeHoldCommand(&stateManager), SustainedCommands(&stateManager), [this] {
	// 	return grabber.isAlgaeIn();
	// }));

	oprtr.Back().WhileTrue(EndPositionCommands(&stateManager));
	oprtr.Back().OnFalse(climber.setClimberClimbedCommand(ClimberConstants::ClimberClosed));

	oprtr.Start().WhileTrue(frc2::cmd::RunOnce([this] {
		climber.setOffset();
	}));

	//Maybe si lo usamos
	// increaseOffsetX.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::setModifyXTarget(0.02);
	//     frc::SmartDashboard::PutBoolean("IncreaseOffset/IncreaseOffsetX", false);
	// }));

	// decreaseOffsetX.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::setModifyXTarget(-0.02);
	//     frc::SmartDashboard::PutBoolean("DecreaseOffset/DecreaseOffsetX", false);
	// }));

	// increaseOffsetLeft.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::setModifyLeftTarget(0.03);
	//     frc::SmartDashboard::PutBoolean("IncreaseOffset/IncreaseOffsetLeft", false);
	// }));

	// decreaseOffsetLeft.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::setModifyLeftTarget(-0.03);
	//     frc::SmartDashboard::PutBoolean("DecreaseOffset/DecreaseOffsetLeft", false);
	// }));

	// increaseOffsetRight.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::setModifyRightTarget(0.03);
	//     frc::SmartDashboard::PutBoolean("IncreaseOffset/IncreaseOffsetRight", false);
	// }));

	// decreaseOffsetRight.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::setModifyRightTarget(-0.03);
	//     frc::SmartDashboard::PutBoolean("DecreaseOffset/DecreaseOffsetRight", false);
	// }));

	// resetOffsets.OnTrue(frc2::cmd::RunOnce([this] {
	//     AlignSpeedHelper::resetOffset();
	//     frc::SmartDashboard::PutBoolean("ResetOffset", false);
	// }));

}

void RobotContainer::ConfigMixedBindigs() {
	(driver.POVDown() && console.Button(12)).OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::Sequence(PassCommandAlign(&stateManager), frc2::cmd::Wait(0.5_s),
				L2Command(&stateManager, &alignManager)), leftAlignPos(&alignManager)).BeforeStarting(
					frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	(driver.POVDown() && console.Button(5)).OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::Sequence(PassCommandAlign(&stateManager), frc2::cmd::Wait(0.5_s),
				L2Command(&stateManager, &alignManager)), rightAlignPos(&alignManager)).BeforeStarting(
					frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	(driver.POVDown() && console.Button(7)).OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::Sequence(PassCommandAlign(&stateManager), frc2::cmd::Wait(0.5_s),
				L3Command(&stateManager, &alignManager)), leftAlignPos(&alignManager)).BeforeStarting(
					frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	(driver.POVDown() && console.Button(8)).OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::Sequence(PassCommandAlign(&stateManager), frc2::cmd::Wait(0.5_s),
				L3Command(&stateManager, &alignManager)), rightAlignPos(&alignManager)).BeforeStarting(
					frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	(driver.POVDown() && console.Button(10)).OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::Sequence(PassCommandAlign(&stateManager), frc2::cmd::Wait(0.5_s),
				L4Command(&stateManager, &alignManager)), leftAlignPos(&alignManager)).BeforeStarting(
					frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	(driver.POVDown() && console.Button(11)).OnTrue(
		frc2::cmd::Parallel(
			frc2::cmd::Sequence(PassCommandAlign(&stateManager), frc2::cmd::Wait(0.5_s),
				L4Command(&stateManager, &alignManager)), rightAlignPos(&alignManager)).BeforeStarting(
					frc2::cmd::RunOnce([this] {
		alignManager.setHeading();
	})).Unless([this] {
		return grabber.isAlgaeIn();
	}));

	console.Button(3).OnTrue(arm.setArmZero());

	console.Button(2).WhileTrue(AlgaeHighManualCommand(&stateManager));
	console.Button(2).OnFalse(
		frc2::cmd::Either(AlgaeHoldCommand(&stateManager), SustainedCommands(&stateManager), [this] {
		return grabber.isAlgaeIn();
	}));

	console.Button(1).WhileTrue(AlgaeLowManualCommand(&stateManager));
	console.Button(1).OnFalse(
		frc2::cmd::Either(AlgaeHoldCommand(&stateManager), SustainedCommands(&stateManager), [this] {
		return grabber.isAlgaeIn();
	}));

	console.Button(6).WhileTrue(frc2::cmd::RunOnce([this] {
		climber.setOffset();
	}));

	console.Button(9).WhileTrue(ProcessorCommand(&stateManager));
	// console.Button(9).OnFalse(
	// 	frc2::cmd::Either(AlgaeHoldCommand(&stateManager), SustainedConfirmedCommands(&stateManager), [this] {
	// 	return grabber.isAlgaeIn();
	// }));

	console.Button(4).WhileTrue(EndPositionCommands(&stateManager));
	console.Button(4).OnFalse(climber.setClimberClimbedCommand(ClimberConstants::ClimberClosed));
}

void RobotContainer::ConfigDefaultCommands() {
	// startCommands.OnTrue(stateManager.setStatePosition());
}

void RobotContainer::ConfigCharacterizationBindings() {
	//-920 descansa toda la partida
	// -570 horizonte para escalar
	//1600 para escalado

	// test.A().WhileTrue(climber.setClimberCommand(165_deg));
	// test.A().OnFalse(climber.setClimberCommand(4_deg));

	// test.B().WhileTrue(climber.setClimberCommand(70_deg));
	// test.B().OnFalse(climber.setClimberCommand(235_deg));

	// test.Y().WhileTrue(climber.setClimberCommand(850_deg));
}

AprilTags::Config RobotContainer::railCameraRight() {
	AprilTags::Config config;
	config.cameraName = "RailRight";
	config.cameraToRobot = { 11.2_in, 3.5_in, 7.752224_in, {0_deg, -15_deg, -47.981360_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

//Climbers no utilizaremos
AprilTags::Config RobotContainer::climberCameraLeft() {
	AprilTags::Config config;
	config.cameraName = "ClimberLeft";
	config.cameraToRobot = { -11.250259_in, 11.154470_in, 7.327807_in, {0_deg, -24_deg, -140.780604_deg} };
	config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m} };
	return config;
}

AprilTags::Config RobotContainer::climberCameraRight() {
	AprilTags::Config config;
	config.cameraName = "ClimberRight";
	config.cameraToRobot = { -9.5_in, -0.819890_in, 7.543_in, {0_deg, -15_deg, -148.525051_deg} };
	return config;
}

AprilTags::Config RobotContainer::railCameraLeft() {
	AprilTags::Config config;
	config.cameraName = "RailLeft";
	config.cameraToRobot = { 8_in, 9.2_in, 11.252224_in, {0_deg, -5_deg, -29.993788_deg} };
	return config;
}
void RobotContainer::UpdateTelemetry() {
	chassis.shuffleboardPeriodic();

	frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());

}
