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

    driver.LeftTrigger().WhileTrue(stateManager.setState(Positions::Intake));
    driver.LeftTrigger().OnFalse(stateManager.setState(Positions::SustainedPosition));

    driver.RightBumper().WhileTrue(ConfirmCommand(&stateManager));
    driver.RightBumper().OnFalse(stateManager.setState(Positions::SustainedPosition));

    driver.POVLeft().WhileTrue(AlgaeCommand(&stateManager));
    driver.POVLeft().OnFalse(stateManager.setState(Positions::SustainedPosition));

    driver.POVUp().WhileTrue(stateManager.setState(Positions::L1Position));
    driver.POVUp().OnFalse(stateManager.setState(Positions::SustainedPosition));

    //driver.POVRight().WhileTrue(/*Coral detection later */);
    //driver.POVRight().OnFalse(stateManager.setState(Positions::SustainedPosition));

}

void RobotContainer::ConfigOperatorBindings() {

    oprtr.LeftBumper().WhileTrue(stateManager.setState(Positions::ProcessorPosition));
    oprtr.LeftBumper().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.RightBumper().WhileTrue(stateManager.setState(Positions::IntakeCoralStation));
    oprtr.RightBumper().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.A().WhileTrue(stateManager.setState(Positions::L1Position));
    oprtr.A().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.B().WhileTrue(stateManager.setState(Positions::L2Front));
    oprtr.B().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.X().WhileTrue(stateManager.setState(Positions::L3Front));
    oprtr.X().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.Y().WhileTrue(stateManager.setState(Positions::L4Front));
    oprtr.Y().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.POVUp().WhileTrue(stateManager.setState(Positions::AlgaeHighReef));
    oprtr.POVUp().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.POVDown().WhileTrue(stateManager.setState(Positions::AlgaeLowReef));
    oprtr.POVDown().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.Back().WhileTrue(stateManager.setState(Positions::EndPosition));
    oprtr.Back().OnFalse(stateManager.setState(Positions::SustainedPosition));

    oprtr.Start().WhileTrue(frc2::cmd::RunOnce([this] {
        climber.setOffset();
    }));
    oprtr.Start().OnFalse(stateManager.setState(Positions::SustainedPosition));

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
    (driver.POVDown() && console.Button(12)).OnTrue(L2Command(&stateManager).AlongWith(leftAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(5)).OnTrue(L2Command(&stateManager).AlongWith(rightAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(7)).OnTrue(L3Command(&stateManager).AlongWith(leftAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(8)).OnTrue(L3Command(&stateManager).AlongWith(rightAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(10)).OnTrue(L4Command(&stateManager).AlongWith(leftAlignPos(&alignManager)));

    (driver.POVDown() && console.Button(11)).OnTrue(L4Command(&stateManager).AlongWith(rightAlignPos(&alignManager)));

    //Maybe es 2 en el numero de la consola :V
    (driver.POVDown() && console.Button(1)).OnTrue(
            AlgaeReefCommand(&stateManager).AlongWith(algaeAlignPos(&alignManager)));

    (!driver.LeftTrigger() && console.AxisMagnitudeGreaterThan(0, 0.1)).OnTrue(
            stateManager.setState(Positions::IntakeCoralStation));
    console.AxisMagnitudeGreaterThan(0, 0.1).OnFalse(stateManager.setState(Positions::SustainedPosition));

    (!driver.LeftTrigger() && console.Button(9)).OnTrue(stateManager.setState(Positions::ProcessorPosition));
    console.Button(9).OnFalse(stateManager.setState(Positions::SustainedPosition));

    console.Button(4).OnTrue(stateManager.setState(Positions::EndPosition));

    driver.POVDown().OnFalse(stateManager.setState(Positions::SustainedPosition));
}

void RobotContainer::ConfigDefaultCommands() {

}

void RobotContainer::ConfigCharacterizationBindings() {

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
