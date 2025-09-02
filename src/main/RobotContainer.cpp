// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("AutoChooser", &autoChooser);
    ConfigureBindings();
    chassis.setAcceptingVisionMeasurements(true);
    frc::DriverStation::SilenceJoystickConnectionWarning(true);

    // pathplanner::NamedCommands::registerCommand("L4Left",
    //         std::move(L4Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager))));

    // pathplanner::NamedCommands::registerCommand("L4Right",
    //         std::move(L4Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager))));

    // pathplanner::NamedCommands::registerCommand("AlgaeReef",
    //         std::move(AlgaeReefCommand(&stateManager, &alignManager).AlongWith(algaeAlignPos(&alignManager))));

    // pathplanner::NamedCommands::registerCommand("Sustained",
    //         std::move(stateManager.setStatePosition(Positions::SustainedPosition)));

    // pathplanner::NamedCommands::registerCommand("Confirm", std::move(ConfirmCommand(&stateManager)));

    // pathplanner::NamedCommands::registerCommand("Intake", std::move(stateManager.setStatePosition(Positions::Intake)));

    // pathplanner::NamedCommands::registerCommand("AlgaeCommand", std::move(AlgaeCommand(&stateManager)));

    // pathplanner::NamedCommands::registerCommand("AlgaeHold",
    //         std::move(stateManager.setStatePosition(Positions::AlgaeHold)));

    // pathplanner::NamedCommands::registerCommand("CoralHold",
    //         std::move(stateManager.setStatePosition(Positions::CoralHold)));

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
    // chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());
    // driver.Back().OnTrue(ResetHeading(&chassis));

    // driver.LeftTrigger().WhileTrue(stateManager.setStatePosition(Positions::Intake));
    // driver.LeftTrigger().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // driver.RightBumper().WhileTrue(ConfirmCommand(&stateManager));
    // driver.RightBumper().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // driver.POVLeft().WhileTrue(AlgaeCommand(&stateManager));
    // driver.POVLeft().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // driver.POVUp().WhileTrue(stateManager.setStatePosition(Positions::L1Position));
    // driver.POVUp().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    //driver.POVRight().WhileTrue(/*Coral detection later */);
    //driver.POVRight().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

}

void RobotContainer::ConfigOperatorBindings() {

    // oprtr.LeftBumper().WhileTrue(stateManager.setStatePosition(Positions::ProcessorPosition));
    // oprtr.LeftBumper().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.RightBumper().WhileTrue(stateManager.setStatePosition(Positions::IntakeCoralStation));
    // oprtr.RightBumper().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.A().WhileTrue(stateManager.setStatePosition(Positions::L1Position));
    // oprtr.A().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.B().WhileTrue(stateManager.setStatePosition(Positions::L2Front));
    // oprtr.B().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.X().WhileTrue(stateManager.setStatePosition(Positions::L3Front));
    // oprtr.X().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.Y().WhileTrue(stateManager.setStatePosition(Positions::L4Front));
    // oprtr.Y().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.POVUp().WhileTrue(stateManager.setStatePosition(Positions::AlgaeHighReef));
    // oprtr.POVUp().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.POVDown().WhileTrue(stateManager.setStatePosition(Positions::AlgaeLowReef));
    // oprtr.POVDown().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.Back().WhileTrue(stateManager.setStatePosition(Positions::EndPosition));
    // oprtr.Back().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // oprtr.Start().WhileTrue(frc2::cmd::RunOnce([this] {
    //     climber.setOffset();
    // }));
    // oprtr.Start().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // endToInitial.OnTrue(frc2::cmd::RunOnce([this] {
    //     stateManager.setStatePosition(Positions::EndPosition);
    //     frc::SmartDashboard::PutBoolean("EndToInitial", false);
    // }));

    // emergency.OnTrue(frc2::cmd::RunOnce([this] {
    //     EmergencyCommand(&stateManager, &intake, &arm, &elevator, &grabber, &climber);
    //     frc::SmartDashboard::PutBoolean("EMERGENCY", false);
    // }));

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
    // (driver.POVDown() && console.Button(12)).OnTrue(
    //         L2Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager)));

    // (driver.POVDown() && console.Button(5)).OnTrue(
    //         L2Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager)));

    // (driver.POVDown() && console.Button(7)).OnTrue(
    //         L3Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager)));

    // (driver.POVDown() && console.Button(8)).OnTrue(
    //         L3Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager)));

    // (driver.POVDown() && console.Button(10)).OnTrue(
    //         L4Command(&stateManager, &alignManager).AlongWith(leftAlignPos(&alignManager)));

    // (driver.POVDown() && console.Button(11)).OnTrue(
    //         L4Command(&stateManager, &alignManager).AlongWith(rightAlignPos(&alignManager)));

    // //Maybe es 2 en el numero de la consola :V
    // (driver.POVDown() && console.Button(1)).OnTrue(
    //         AlgaeReefCommand(&stateManager, &alignManager).AlongWith(algaeAlignPos(&alignManager)));

    // (!driver.LeftTrigger() && console.AxisMagnitudeGreaterThan(0, 0.1)).OnTrue(
    //         stateManager.setStatePosition(Positions::IntakeCoralStation));
    // console.AxisMagnitudeGreaterThan(0, 0.1).OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // (!driver.LeftTrigger() && console.Button(9)).OnTrue(stateManager.setStatePosition(Positions::ProcessorPosition));
    // console.Button(9).OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));

    // console.Button(4).OnTrue(stateManager.setStatePosition(Positions::EndPosition));

    // driver.POVDown().OnFalse(stateManager.setStatePosition(Positions::SustainedPosition));
}

void RobotContainer::ConfigDefaultCommands() {

}

void RobotContainer::ConfigCharacterizationBindings() {
    test.A().WhileTrue(arm.setCharacterization(90.0_deg));
    test.A().OnFalse(arm.setCharacterization(0.0_deg));

    test.B().WhileTrue(elevator.setCharacterization(50.0_m));
    test.B().OnFalse(elevator.setCharacterization(0.0_m));

    test.X().WhileTrue(intake.setCharacterization(1.0_V, 1.0_V, 45.0_deg));
    test.X().OnFalse(intake.setCharacterization(0.0_V, 0.0_V, 0.0_deg));

    test.Y().WhileTrue(grabber.setCharacterization(5_V));
    test.Y().OnFalse(grabber.setCharacterization(0.0_V));

    //test.Y().WhileTrue(climber.setCharacterization(20.0_deg));
    //test.Y().OnFalse(climber.setCharacterization(0.0_deg));

}

AprilTags::Config RobotContainer::railCameraLeft() {
    AprilTags::Config config;
    config.cameraName = "RailLeft";
    config.cameraToRobot = {7.200000_in, -5.892500_in, 6.368259_in, {0_deg, -21.500115_deg, 30.026518_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::climberCameraLeft() {
    AprilTags::Config config;
    config.cameraName = "ClimberLeft";
    config.cameraToRobot = {6.000000_in, 11.000000_in, 7.752224_in, {0_deg, -21.000118_deg, 25.025948_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::climberCameraRight() {
    AprilTags::Config config;
    config.cameraName = "ClimberRight";
    config.cameraToRobot = {11.000000_in, -7.000000_in, 9.752224_in, {0_deg, -15.000170_deg, 50.018714_deg}};
    return config;
}

AprilTags::Config RobotContainer::railCameraRight() {
    AprilTags::Config config;
    config.cameraName = "RailRight";
    config.cameraToRobot = {-9.648405_in, 8.631463_in, 8.410513_in, {0_deg, -28.125_deg, 120_deg}};
    return config;
}
void RobotContainer::UpdateTelemetry() {
    chassis.shuffleboardPeriodic();
    //driver.updateTelemetry();
    //oprtr.updateTelemetry();
    //console.updateTelemetry();

    frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());

}
