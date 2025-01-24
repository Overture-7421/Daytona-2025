// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    frc::SmartDashboard::PutData("AutoChooser", &autoChooser);

    ConfigureBindings();

    chassis.setAcceptingVisionMeasurements(true);
}

void RobotContainer::ConfigureBindings() {
    ConfigDriverBindings();
    ConfigOperatorBindings();
    ConfigDefaultCommands();
    //ConfigCharacterizationBindings();
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    chassis.setAcceptingVisionMeasurements(true);

    return autoChooser.GetSelected();
}

void RobotContainer::ConfigDriverBindings() {
    chassis.SetDefaultCommand(DriveCommand(&chassis, &driver).ToPtr());

    driver.Back().OnTrue(ResetHeading(&chassis));

    //driver.X().OnTrue(arm.setArmCommand(0_deg, 0_deg));
    //driver.B().OnTrue(arm.setArmCommand(-30_deg, -30_deg));
    //driver.Y().OnTrue(arm.setArmCommand(30_deg, 30_deg));

    driver.RightBumper().OnTrue(elevator.setElevatorCommand(0.50_m));
    driver.LeftBumper().OnTrue(elevator.setElevatorCommand(0.0_m));

    driver.A().WhileTrue(HighAlgae(&arm, &elevator, &chassis));

    driver.X().WhileTrue(climber.setClimberCommand(90_deg));
    driver.B().WhileTrue(climber.setClimberCommand(-90_deg));
    driver.Y().WhileTrue(climber.setClimberCommand(0_deg));



    /*
    driver.RightBumper().WhileTrue(intake.moveIntake(6_V));
    driver.LeftBumper().WhileTrue(intake.moveIntake(-6_V));

    driver.A().WhileTrue(intake.moveIntake(0_V));

    driver.B().WhileTrue(intake.moveIntake(11_V));
    driver.X().WhileTrue(intake.moveIntake(-11_V));

    driver.Y().WhileTrue(intake.moveIntake(-2_V));
    */


}

void RobotContainer::ConfigOperatorBindings() {
}

void RobotContainer::ConfigDefaultCommands() {

}

void RobotContainer::ConfigCharacterizationBindings() {
}

AprilTags::Config RobotContainer::testCameraConfig() {
    AprilTags::Config config;
    config.cameraName = "Global_Shutter_Camera";
    config.cameraToRobot = {14.950771_in, 0_m, 14.034697_in, {0_deg, 0_deg, 0_deg}};
    return config;
}

void RobotContainer::UpdateTelemetry() {
    chassis.shuffleboardPeriodic();
}
