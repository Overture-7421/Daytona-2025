// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

    pathplanner::NamedCommands::registerCommand("coralL4", std::move(
    frc2::cmd::Sequence(
      L4Command(&arm, &elevator),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::L4Position) && arm.isArmAtPosition(ArmConstants::ArmL4Reef, ArmConstants::WristL4Reef);}),
      intake.moveIntake(IntakeConstants::CoralRelease).WithTimeout(0.5_s),
      ClosedCommand(&arm, &elevator)
    )));

    pathplanner::NamedCommands::registerCommand("coralL1", std::move(
    frc2::cmd::Sequence(
      L1Command(&arm, &elevator),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::L1Position) && arm.isArmAtPosition(ArmConstants::ArmL1Reef, ArmConstants::WristL1Reef);})
    )));

    pathplanner::NamedCommands::registerCommand("lowAlgae", std::move(
    frc2::cmd::Sequence(
      LowAlgae(&arm, &elevator),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::LowAlgae) && arm.isArmAtPosition(ArmConstants::ArmLowAlgae, ArmConstants::WristLowAlgae);}),
      intake.moveIntake(IntakeConstants::AlgeaGrab).WithTimeout(0.5_s),
      ClosedCommand(&arm, &elevator)
    )));

    pathplanner::NamedCommands::registerCommand("highAlgae", std::move(
    frc2::cmd::Sequence(
      HighAlgae(&arm, &elevator),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::HighAlgae) && arm.isArmAtPosition(ArmConstants::ArmHighAlgae, ArmConstants::WristHighAlgae);}),
      intake.moveIntake(IntakeConstants::AlgeaGrab).WithTimeout(0.5_s),
      ClosedCommand(&arm, &elevator)
    )));

    pathplanner::NamedCommands::registerCommand("processor", std::move(
    frc2::cmd::Sequence(
      Processor(&arm, &elevator),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::ProcessorPosition) && arm.isArmAtPosition(ArmConstants::ArmProcessor, ArmConstants::WristProcessor);}),
      intake.moveIntake(IntakeConstants::AlgeaRelease).WithTimeout(0.5_s),
      ClosedCommand(&arm, &elevator)
    )));

    pathplanner::NamedCommands::registerCommand("algaeNet", std::move(
    frc2::cmd::Sequence(
      NetCommand(&arm, &elevator),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::NetPosition) && arm.isArmAtPosition(ArmConstants::ArmNet, ArmConstants::WristNet);}),
      intake.moveIntake(IntakeConstants::AlgeaRelease).WithTimeout(0.5_s),
      ClosedCommand(&arm, &elevator)
    )));

    pathplanner::NamedCommands::registerCommand("coralStation", std::move(
    frc2::cmd::Sequence(
      SourceCommand(&arm, &elevator, &intake),
      frc2::cmd::WaitUntil([&] {return elevator.isElevatorAtPosition(ElevatorConstants::CoralStationPosition) && arm.isArmAtPosition(ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation);}),
      ClosedCommand(&arm, &elevator)
    )));

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

    driver.A().OnTrue(intake.setIntakeCommand(12_V, -90_deg));
    driver.A().OnFalse(intake.setIntakeCommand(0_V, 0_deg));

    driver.B().OnTrue(intake.setIntakeCommand(12_V, -180_deg));
    driver.B().OnFalse(intake.setIntakeCommand(0_V, 0_deg));

}

void RobotContainer::ConfigOperatorBindings() {

    oprtr.X().WhileTrue(intake.moveIntake(-8_V));
    oprtr.Y().WhileTrue(intake.moveIntake(8_V));

    oprtr.LeftBumper().WhileTrue(intake.moveIntake(-7_V));
    oprtr.B().WhileTrue(intake.moveIntake(-6_V));

    oprtr.RightBumper().WhileTrue(intake.moveIntake(-4_V));

    oprtr.A().OnTrue(intake.moveIntake(0_V));

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
