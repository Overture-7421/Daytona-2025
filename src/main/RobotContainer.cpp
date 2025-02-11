// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/Commands.h>
#include "Commands/NetCommand/NetPose.h"

RobotContainer::RobotContainer() {

    pathplanner::NamedCommands::registerCommand("closed",
            std::move(frc2::cmd::Sequence(ClosedCommand(&arm, &elevator, &intake, &superStructure))));

    pathplanner::NamedCommands::registerCommand("coralL4",
            std::move(
                    frc2::cmd::Sequence(L4Command(&arm, &elevator, &superStructure))));

        pathplanner::NamedCommands::registerCommand("spitCoral", std::move(frc2::cmd::Parallel(intake.setIntakeCommand(IntakeConstants::CoralRelease, IntakeConstants::JawCoralOpen),
                            superStructure.setState(SuperStructureStates::SpitCoral))));

        pathplanner::NamedCommands::registerCommand("spitAlgae", std::move(frc2::cmd::Sequence(intake.setIntakeCommand(IntakeConstants::CoralRelease, IntakeConstants::JawCoralOpen),
                            superStructure.setState(SuperStructureStates::SpitAlgae))));     

    pathplanner::NamedCommands::registerCommand("coralL1",
            std::move(frc2::cmd::Sequence(L1Command(&arm, &elevator, &superStructure))));

    pathplanner::NamedCommands::registerCommand("lowAlgae",
            std::move(
                    frc2::cmd::Sequence(LowAlgae(&arm, &elevator, &intake, &superStructure),
                            intake.setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae),
                            superStructure.setState(SuperStructureStates::HoldAlgae).WithTimeout(0.3_s),
                            ClosedCommand(&arm, &elevator, &intake, &superStructure))));

        pathplanner::NamedCommands::registerCommand("spitAlgae",
            std::move(
                    frc2::cmd::Sequence(intake.setIntakeCommand(IntakeConstants::AlgaeGrab, IntakeConstants::JawAlgae),
                            superStructure.setState(SuperStructureStates::HoldAlgae))));

    pathplanner::NamedCommands::registerCommand("highAlgae",
            std::move(
                    frc2::cmd::Sequence(HighAlgae(&arm, &elevator, &intake, &superStructure))));

    pathplanner::NamedCommands::registerCommand("processor",
            std::move(
                    frc2::cmd::Sequence(Processor(&arm, &elevator, &superStructure))));

    pathplanner::NamedCommands::registerCommand("algaeNet",
            std::move(
                    frc2::cmd::Sequence(NetCommand(&arm, &elevator, &superStructure))));

    pathplanner::NamedCommands::registerCommand("coralStation",
            std::move(
                    frc2::cmd::Sequence(SourceCommand(&arm, &elevator, &intake, &superStructure))));

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

    driver.Y().WhileTrue(
            NetCommand(&arm, &elevator, &superStructure).AlongWith(AlignToNet(&chassis, NetPose::pose).ToPtr()));
    driver.Y().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.B().WhileTrue(SourceCommand(&arm, &elevator, &intake, &superStructure));
    driver.B().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.X().WhileTrue(CoralGroundGrabCommand(&arm, &elevator, &intake, &superStructure));
    driver.X().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.A().WhileTrue(AlgaeGroundGrabCommand(&arm, &elevator, &intake, &superStructure));
    driver.A().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.RightBumper().WhileTrue(SpitGamePiece(&intake, &superStructure));
    driver.RightBumper().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    //Palllet Execute Aligns

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

    oprtr.A().WhileTrue(L1Command(&arm, &elevator, &superStructure));
    oprtr.A().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.B().WhileTrue(L2Command(&arm, &elevator, &superStructure));
    oprtr.B().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.X().WhileTrue(L3Command(&arm, &elevator, &superStructure));
    oprtr.X().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.Y().WhileTrue(L4Command(&arm, &elevator, &superStructure));
    oprtr.Y().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.RightTrigger().WhileTrue(SourceCommand(&arm, &elevator, &intake, &superStructure));
    oprtr.RightTrigger().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.POVDown().WhileTrue(LowAlgae(&arm, &elevator, &intake, &superStructure));
    oprtr.POVDown().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.POVUp().WhileTrue(HighAlgae(&arm, &elevator, &intake, &superStructure));
    oprtr.POVUp().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.LeftBumper().WhileTrue(Processor(&arm, &elevator, &superStructure));
    oprtr.LeftBumper().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.Start().WhileTrue(climber.setClimberCommand(ClimberConstants::OpenPosition));
    oprtr.Start().OnFalse(climber.setClimberCommand(ClimberConstants::ClosedPosition));

}

void RobotContainer::ConfigMixedBindigs() {
    (console.Button(0) && driver.POVRight()).OnTrue(
            L1Command(&arm, &elevator, &superStructure).AlongWith(centerAlignPos(&chassis, &tagLayout)));

    (console.Button(1) && driver.POVRight()).OnTrue(
            L2Command(&arm, &elevator, &superStructure).AlongWith(leftAlignPos(&chassis, &tagLayout)));
    (console.Button(2) && driver.POVRight()).OnTrue(
            L2Command(&arm, &elevator, &superStructure).AlongWith(rightAlignPos(&chassis, &tagLayout)));

    (console.Button(3) && driver.POVRight()).OnTrue(
            L3Command(&arm, &elevator, &superStructure).AlongWith(leftAlignPos(&chassis, &tagLayout)));
    (console.Button(4) && driver.POVRight()).OnTrue(
            L3Command(&arm, &elevator, &superStructure).AlongWith(rightAlignPos(&chassis, &tagLayout)));

    (console.Button(5) && driver.POVRight()).OnTrue(
            L4Command(&arm, &elevator, &superStructure).AlongWith(leftAlignPos(&chassis, &tagLayout)));
    (console.Button(6) && driver.POVRight()).OnTrue(
            L4Command(&arm, &elevator, &superStructure).AlongWith(rightAlignPos(&chassis, &tagLayout)));

    (console.Button(7) && driver.POVRight()).OnTrue(
            LowAlgae(&arm, &elevator, &intake, &superStructure).AlongWith(centerAlignPos(&chassis, &tagLayout)));
    (console.Button(8) && driver.POVRight()).OnTrue(
            HighAlgae(&arm, &elevator, &intake, &superStructure).AlongWith(centerAlignPos(&chassis, &tagLayout)));

    (console.Button(9) && driver.POVRight()).OnTrue(
            SourceCommand(&arm, &elevator, &intake, &superStructure).AlongWith(stationPos(&chassis, &tagLayout)));

    (console.Button(10) && driver.POVRight()).OnTrue(
            Processor(&arm, &elevator, &superStructure).AlongWith(processorPos(&chassis, &tagLayout)));

    (console.Button(11) && driver.POVRight()).OnTrue(climber.setClimberCommand(ClimberConstants::ClosedPosition)); //Position is not defined yet

    driver.POVRight().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

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
