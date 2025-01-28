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

    // driver.Back().OnTrue(ResetHeading(&chassis));

    // driver.RightBumper().OnTrue(elevator.setElevatorCommand(0.30_m));
    //  driver.LeftBumper().OnTrue(elevator.setElevatorCommand(0.0_m));

    //driver.A().WhileTrue(HighAlgae(&arm, &elevator, &chassis));

}

void RobotContainer::ConfigOperatorBindings() {

    /*
     oprtr.X().WhileTrue(intake.moveIntake(-8_V));
     oprtr.Y().WhileTrue(intake.moveIntake(8_V));

     oprtr.B().WhileTrue(intake.moveIntake(-6_V));

     oprtr.RightBumper().WhileTrue(intake.moveIntake(-4_V));


     oprtr.A().OnTrue(intake.moveIntake(0_V));

     */

}

void RobotContainer::ConfigDefaultCommands() {

}

void RobotContainer::ConfigCharacterizationBindings() {
}

AprilTags::Config RobotContainer::frontRightCamera() {
    AprilTags::Config config;
    config.cameraName = "Global_Shutter_Camera";
    config.cameraToRobot = {10.5408175_in, -9.8155955_in, 8.358231_in, {0_deg, -28.125_deg, -30_deg}};
    return config;
}

AprilTags::Config RobotContainer::frontLeftCamera() {
    AprilTags::Config config;
    config.cameraName = "Global_Shutter_Camera_2";
    config.cameraToRobot = {10.5408175_in, 9.8155955_in, 8.358231_in, {0_deg, -28.125_deg, 30_deg}};
    return config;
}

AprilTags::Config RobotContainer::backRightCamera() {
    AprilTags::Config config;
    config.cameraName = "Global_Shutter_Camera_3";
    config.cameraToRobot = {-10.5408175_in, -9.8155955_in, 8.358231_in, {0_deg, -28.125_deg, 150_deg}};
    return config;
}

AprilTags::Config RobotContainer::backLeftCamera() {
    AprilTags::Config config;
    config.cameraName = "Global_Shutter_Camera_4";
    config.cameraToRobot = {-10.5408175_in, -9.8155955_in, 8.358231_in, {0_deg, -28.125_deg, -150_deg}};
    return config;
}
void RobotContainer::UpdateTelemetry() {
    chassis.shuffleboardPeriodic();
}
