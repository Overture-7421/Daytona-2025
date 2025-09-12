// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {

    // autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    // frc::SmartDashboard::PutData("AutoChooser", &autoChooser);
    ConfigureBindings();
    chassis.setAcceptingVisionMeasurements(true);
    frc::DriverStation::SilenceJoystickConnectionWarning(true);

    pathplanner::NamedCommands::registerCommand("FirstL4", std::move(L4CommandAuto(&stateManager, &alignManager)));

    pathplanner::NamedCommands::registerCommand("LeftAlign", std::move(leftAlignPos(&alignManager)));

    pathplanner::NamedCommands::registerCommand("RightAlign", std::move(rightAlignPos(&alignManager)));

    pathplanner::NamedCommands::registerCommand("L4", std::move(L4Command(&stateManager, &alignManager)));

    // pathplanner::NamedCommands::registerCommand("Confirm", std::move(ConfirmCommand(&stateManager)));

    // pathplanner::NamedCommands::registerCommand("Intake", std::move(stateManager.setStatePosition(Positions::Intake)));

    // pathplanner::NamedCommands::registerCommand("AlgaeCommand", std::move(AlgaeCommand(&stateManager)));

    // pathplanner::NamedCommands::registerCommand("AlgaeHold",
    // 	std::move(stateManager.setStatePosition(Positions::AlgaeHold)));

    // pathplanner::NamedCommands::registerCommand("CoralHold",
    // 	std::move(stateManager.setStatePosition(Positions::CoralHold)));

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

    driver.LeftTrigger().WhileTrue(stateManager.SustainedToIntake().AndThen(L1Command(&stateManager)));
    driver.LeftTrigger().OnFalse(stateManager.L1PositionToCoralHold());
    // driver.LeftTrigger().WhileTrue(
    // frc2::cmd::Parallel(intake.setCharacterization(3_V, 8.25_V, 131_deg), arm.setCharacterization(-90_deg)));
    // driver.LeftTrigger().OnFalse(intake.setCharacterization(0_V, 0_V, 30_deg));

    // driver.RightBumper().WhileTrue(intake.setCharacterization(-6_V, -6_V, 30_deg));
    // driver.RightBumper().OnFalse(intake.setCharacterization(0_V, 0_V, 30_deg));

    // driver.LeftBumper().WhileTrue(CharacterizationCommand(&intake, &arm, &elevator, &grabber, &climber));
    // driver.LeftBumper().OnFalse(ClosedCommand(&intake, &arm, &elevator, &grabber, &climber));

    // driver.LeftTrigger().WhileTrue(stateManager.setStatePosition(Positions::Intake));
    // driver.A().WhileTrue(stateManager.setStatePosition(Positions::SustainedPosition));

    driver.RightBumper().WhileTrue(ConfirmCommand(&stateManager, &grabber));
    driver.RightBumper().OnFalse(SustainedCommands(&stateManager));

    driver.POVLeft().WhileTrue(AlgaeGroundCommand(&stateManager));
    driver.POVLeft().OnFalse(SustainedCommands(&stateManager));

    driver.POVUp().WhileTrue(L1Command(&stateManager));
    driver.POVUp().OnFalse(SustainedCommands(&stateManager).AndThen(stateManager.L1PositionToCoralHold()));

    driver.POVRight().WhileTrue(NetCommand(&stateManager));
    driver.POVRight().OnFalse(SustainedCommands(&stateManager));

    //driver.A().WhileTrue(leftAlignPos(&alignManager));

    //driver.B().WhileTrue(rightAlignPos(&alignManager));

    armZero.OnTrue(arm.setArmZero().andThen(frc::cmd::RunOnce([this] {
        frc::SmartDashboard::PutBoolean("ARM-ZERO", false);
    })));

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

    oprtr.POVUp().WhileTrue(AlgaeHighManualCommand(&stateManager).AndThen(AlgaeHoldCommand(&stateManager)));
    oprtr.POVUp().OnFalse(SustainedCommands(&stateManager).AndThen(stateManager.AlgaeHighReefToAlgaeHold()));

    oprtr.POVDown().WhileTrue(AlgaeLowManualCommand(&stateManager).AndThen(AlgaeHoldCommand(&stateManager)));
    oprtr.POVDown().OnFalse(SustainedCommands(&stateManager).AndThen(stateManager.AlgaeLowReefToAlgaeHold()));

    oprtr.Back().WhileTrue(EndPositionCommands(&stateManager));
    // oprtr.Back().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    oprtr.Start().WhileTrue(frc2::cmd::RunOnce([this] {
        climber.setOffset();
    }));
    // oprtr.Start().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // endToInitial.OnTrue(frc2::cmd::RunOnce([this] {
    // 	stateManager.setStatePosition(Positions::EndPosition);
    // 	frc::SmartDashboard::PutBoolean("EndToInitial", false);
    // }));

    // emergency.OnTrue(EmergencyCommand(&stateManager, &intake, &arm, &elevator, &grabber, &climber).AndThen(
    // 	frc2::cmd::RunOnce([this] {
    // 	frc::SmartDashboard::PutBoolean("EMERGENCY", false);
    // })));

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
            L2Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(5)).OnTrue(
            L2Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(7)).OnTrue(
            L3Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(8)).OnTrue(
            L3Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(10)).OnTrue(
            L4Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(11)).OnTrue(
            L4Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager)));

    // //Maybe es 2 en el numero de la consola :V
    // (driver.POVDown() && console.Button(1)).OnTrue(
    // AlgaeReefCommand(&stateManager, &alignManager).AlongWith(
    // algaeAlignPos(&alignManager).AndThen(AlgaeHoldCommand(&stateManager))));

    // (!driver.LeftTrigger() && console.AxisMagnitudeGreaterThan(0, 0.1)).OnTrue(
    // 	stateManager.setStatePosition(Positions::IntakeCoralStation));
    // console.AxisMagnitudeGreaterThan(0, 0.1).OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // (!driver.LeftTrigger() && console.Button(9)).OnTrue(stateManager.setStatePosition(Positions::ProcessorPosition));
    // console.Button(9).OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    console.Button(4).OnTrue(EndPositionCommands(&stateManager));

    driver.POVDown().OnFalse(SustainedCommands(&stateManager));
}

void RobotContainer::ConfigDefaultCommands() {
    // startCommands.OnTrue(stateManager.setStatePosition());
}

void RobotContainer::ConfigCharacterizationBindings() {
    //test.A().WhileTrue(arm.setCharacterization(-90.0_deg));
    //test.A().OnFalse(arm.setCharacterization(0.0_deg));

    //test.B().WhileTrue(elevator.setCharacterization(1.00_m));
    //test.B().OnFalse(elevator.setCharacterization(0.05_m));

    //test.X().WhileTrue(intake.setCharacterization(3.0_V, 8.25_V, 132_deg));
    //test.X().OnFalse(intake.setCharacterization(3.0_V, 8.25_V, 15_deg));

    // test.Y().WhileTrue(grabber.setCharacterization(5_V));
    // test.Y().OnFalse(grabber.setCharacterization(0.0_V));

    //test.Y().WhileTrue(CharacterizationCommand(&intake, &arm, &elevator, &grabber, &climber));
    //test.Y().OnFalse(frc2::cmd::Sequence(arm.setCharacterization(-90_deg), elevator.setCharacterization(0.01_m)));

    //test.Y().WhileTrue(climber.setCharacterization(212.0_deg));
    //test.Y().OnFalse(climber.setCharacterization(122.0_deg));

}

AprilTags::Config RobotContainer::railCameraRight() {
    AprilTags::Config config;
    config.cameraName = "RailRight";
    config.cameraToRobot = {11.2_in, 3.5_in, 7.752224_in, {0_deg, -15_deg, -47.981360_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

//Climbers no utilizaremos
AprilTags::Config RobotContainer::climberCameraLeft() {
    AprilTags::Config config;
    config.cameraName = "ClimberLeft";
    config.cameraToRobot = {-11.162644_in, 10.651305_in, 8.845276_in, {0_deg, -20_deg, -119.975294_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::climberCameraRight() {
    AprilTags::Config config;
    config.cameraName = "ClimberRight";
    config.cameraToRobot = {-11.4_in, -1.319890_in, 6.843439_in, {0_deg, -15_deg, -144.981261_deg}};
    return config;
}

AprilTags::Config RobotContainer::railCameraLeft() {
    AprilTags::Config config;
    config.cameraName = "RailLeft";
    config.cameraToRobot = {8_in, 9.2_in, 11.252224_in, {0_deg, -5_deg, -29.993788_deg}};
    return config;
}
void RobotContainer::UpdateTelemetry() {
    chassis.shuffleboardPeriodic();
    //driver.updateTelemetry();
    //oprtr.updateTelemetry();
    //console.updateTelemetry();

    frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());

}
