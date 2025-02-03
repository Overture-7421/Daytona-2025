// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/Commands.h>
#include "Commands/NetCommand/NetPose.h"

RobotContainer::RobotContainer() {

    pathplanner::NamedCommands::registerCommand("coralL4",
            std::move(
                    frc2::cmd::Sequence(L4Command(&arm, &elevator, &intake),
                            intake.setIntakeCommand(IntakeConstants::CoralRelease, IntakeConstants::JawCoralOpen,
                                    IntakeStates::SpitCoral).WithTimeout(0.5_s),
                            ClosedCommand(&arm, &elevator, &intake))));

    pathplanner::NamedCommands::registerCommand("coralL1",
            std::move(frc2::cmd::Sequence(L1Command(&arm, &elevator, &intake))));

    pathplanner::NamedCommands::registerCommand("lowAlgae",
            std::move(
                    frc2::cmd::Sequence(LowAlgae(&arm, &elevator, &intake),
                            intake.setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae,
                                    IntakeStates::HoldAlgae).WithTimeout(0.5_s),
                            ClosedCommand(&arm, &elevator, &intake))));

    pathplanner::NamedCommands::registerCommand("highAlgae",
            std::move(
                    frc2::cmd::Sequence(HighAlgae(&arm, &elevator, &intake),
                            intake.setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae,
                                    IntakeStates::HoldAlgae).WithTimeout(0.5_s),
                            ClosedCommand(&arm, &elevator, &intake))));

    pathplanner::NamedCommands::registerCommand("processor",
            std::move(
                    frc2::cmd::Sequence(Processor(&arm, &elevator, &intake),
                            intake.setIntakeCommand(IntakeConstants::AlgaeRelease, IntakeConstants::JawAlgae,
                                    IntakeStates::SpitAlgae).WithTimeout(0.5_s),
                            ClosedCommand(&arm, &elevator, &intake))));

    pathplanner::NamedCommands::registerCommand("algaeNet",
            std::move(
                    frc2::cmd::Sequence(NetCommand(&arm, &elevator, &intake),
                            intake.setIntakeCommand(IntakeConstants::AlgaeRelease, IntakeConstants::JawAlgae,
                                    IntakeStates::SpitAlgae).WithTimeout(0.5_s),
                            ClosedCommand(&arm, &elevator, &intake))));

    pathplanner::NamedCommands::registerCommand("coralStation",
            std::move(
                    frc2::cmd::Sequence(SourceCommand(&arm, &elevator, &intake),
                            ClosedCommand(&arm, &elevator, &intake))));

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("AutoChooser", &autoChooser);
    ConfigureBindings();
    chassis.setAcceptingVisionMeasurements(true);
}

void RobotContainer::ConfigureBindings() {
    ConfigDriverBindings();
    ConfigOperatorBindings();
    ConfigMixedBindigs();
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

    driver.Y().WhileTrue(NetCommand(&arm, &elevator, &intake).AlongWith(AlignToNet(&chassis, NetPose::pose).ToPtr()));
    driver.Y().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    driver.B().WhileTrue(SourceCommand(&arm, &elevator, &intake));
    driver.B().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    driver.X().WhileTrue(CoralGroundGrabCommand(&arm, &elevator, &intake));
    driver.X().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    driver.A().WhileTrue(AlgaeGroundGrabCommand(&arm, &elevator, &intake));
    driver.A().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    driver.Start().WhileTrue(SpitGamePiece(&intake));
    driver.Start().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    //Palllet Execute Aligns

    driver.LeftBumper().WhileTrue(intake.setIntakeCommand(12_V, 100_deg, IntakeStates::EnterAlgae));
    driver.LeftBumper().OnFalse(intake.setIntakeCommand(0_V, 0_deg, IntakeStates::HoldCoral));

    driver.RightBumper().WhileTrue(intake.setIntakeCommand(6_V, 10_deg, IntakeStates::EnterCoral));
    driver.RightBumper().OnFalse(intake.setIntakeCommand(0_V, 0_deg, IntakeStates::HoldCoral));

}

void RobotContainer::ConfigOperatorBindings() {

    /* Intake Testing
     oprtr.X().WhileTrue(intake.moveIntake(-8_V));
     oprtr.Y().WhileTrue(intake.moveIntake(8_V));

     oprtr.LeftBumper().WhileTrue(intake.moveIntake(-7_V));
     oprtr.B().WhileTrue(intake.moveIntake(-6_V));

     oprtr.RightBumper().WhileTrue(intake.moveIntake(-4_V));

     oprtr.A().OnTrue(intake.moveIntake(0_V));
     */

    oprtr.A().WhileTrue(L1Command(&arm, &elevator, &intake));
    oprtr.A().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.B().WhileTrue(L2Command(&arm, &elevator, &intake));
    oprtr.B().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.X().WhileTrue(L3Command(&arm, &elevator, &intake));
    oprtr.X().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.Y().WhileTrue(L4Command(&arm, &elevator, &intake));
    oprtr.Y().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.RightTrigger().WhileTrue(SourceCommand(&arm, &elevator, &intake));
    oprtr.RightTrigger().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.POVDown().WhileTrue(LowAlgae(&arm, &elevator, &intake));
    oprtr.POVDown().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.POVUp().WhileTrue(HighAlgae(&arm, &elevator, &intake));
    oprtr.POVUp().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.LeftBumper().WhileTrue(Processor(&arm, &elevator, &intake));
    oprtr.LeftBumper().OnFalse(ClosedCommand(&arm, &elevator, &intake));

    oprtr.Start().WhileTrue(climber.setClimberCommand(ClimberConstants::OpenPosition));
    oprtr.Start().OnFalse(climber.setClimberCommand(ClimberConstants::ClosedPosition));

}

void RobotContainer::ConfigMixedBindigs() {
}

void RobotContainer::ConfigDefaultCommands() {
    climber.setClimberCommand(ClimberConstants::ClosedPosition);

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
