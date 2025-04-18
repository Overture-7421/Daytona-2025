// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Commands/NetCommand/NetPose.h"

RobotContainer::RobotContainer() {

    pathplanner::NamedCommands::registerCommand("closed",
            std::move(ClosedCommand(&arm, &elevator, &intake, &superStructure)));

    pathplanner::NamedCommands::registerCommand("coralL4",
            (L4Command(&arm, &elevator, &superStructure).WithTimeout(4_s)));

    pathplanner::NamedCommands::registerCommand("spitCoral",
            std::move(SpitGamePiece(&intake, &superStructure, &elevator, &arm)));

    pathplanner::NamedCommands::registerCommand("coralL1",
            std::move(frc2::cmd::Sequence(L1Command(&arm, &elevator, &superStructure))));

    pathplanner::NamedCommands::registerCommand("lowAlgae",
            std::move(
                    frc2::cmd::Sequence(LowAlgae(&arm, &elevator, &intake, &superStructure),
                            superStructure.setState(SuperStructureStates::HoldAlgae))));

    pathplanner::NamedCommands::registerCommand("spitAlgae",
            std::move(
                    frc2::cmd::Parallel(intake.moveIntake(IntakeConstants::AlgaeRelease),
                            superStructure.setState(SuperStructureStates::HoldAlgae))));

    pathplanner::NamedCommands::registerCommand("highAlgae",
            std::move(
                    frc2::cmd::Sequence(HighAlgaeAuto(&arm, &elevator, &intake, &superStructure).WithTimeout(4_s),
                    superStructure.setState(SuperStructureStates::HoldAlgae))));

    pathplanner::NamedCommands::registerCommand("netCommand",
            std::move(
                    frc2::cmd::Sequence(NetCommand(&arm, &elevator, &superStructure))));

    pathplanner::NamedCommands::registerCommand("lowAlgae",
            std::move(
                    frc2::cmd::Sequence(LowAlgaeAuto(&arm, &elevator, &intake, &superStructure).WithTimeout(4_s),
                    superStructure.setState(SuperStructureStates::HoldAlgae))));

    pathplanner::NamedCommands::registerCommand("algaeNet",
            std::move(frc2::cmd::Sequence(NetCommand(&arm, &elevator, &superStructure))));

    pathplanner::NamedCommands::registerCommand("coralStation",
            std::move(
                    frc2::cmd::Sequence(
                            SourceCommandAuto(&arm, &elevator, &intake, &superStructure).BeforeStarting([this]() {
                                arm.changeBlockedWrist(100);
                            }))));

    pathplanner::NamedCommands::registerCommand("confirmCoral",
            std::move(
                    frc2::cmd::Sequence(arm.setArmCommand(ArmConstants::ArmCoralStationAway, ArmConstants::WristClosed),
                            arm.setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                            elevator.setElevatorCommand(ElevatorConstants::ClosedPosition))));

    pathplanner::NamedCommands::registerCommand("rightAlign",
            std::move(rightAlignPos(&chassis, &tagLayout, &driver).WithTimeout(5.0_s)));
    pathplanner::NamedCommands::registerCommand("leftAlign",
            std::move(leftAlignPos(&chassis, &tagLayout, &driver).WithTimeout(5.0_s)));

    pathplanner::NamedCommands::registerCommand("rightAlignFast",
            std::move(rightAlignPos(&chassis, &tagLayout, &driver)));
    pathplanner::NamedCommands::registerCommand("leftAlignFast",
            std::move(leftAlignPos(&chassis, &tagLayout, &driver)));

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

    driver.Y().WhileTrue(NetCommand(&arm, &elevator, &superStructure)); // Align: .AlongWith(AlignToNet(&chassis, NetPose::pose).ToPtr())
    driver.Y().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    //driver.B().WhileTrue(SourceCommand(&arm, &elevator, &intake, &superStructure));
    //driver.B().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.B().WhileTrue(CoralGroundGrabCommandFront(&arm, &elevator, &intake, &superStructure));
    driver.B().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.X().WhileTrue(CoralGroundGrabCommandBack(&arm, &elevator, &intake, &superStructure));
    driver.X().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.A().WhileTrue(AlgaeGroundGrabCommand(&arm, &elevator, &intake, &superStructure));
    driver.A().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.RightBumper().OnTrue(SpitGamePiece(&intake, &superStructure, &elevator, &arm));
    driver.RightBumper().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    driver.LeftBumper().WhileTrue(SpitGamePiece(&intake, &superStructure, &elevator, &arm));
    driver.LeftBumper().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

}

void RobotContainer::ConfigOperatorBindings() {

    //Right Bumper is the nextButton for sourceCommand

    oprtr.A().WhileTrue(L1Command(&arm, &elevator, &superStructure));
    oprtr.A().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.B().WhileTrue(L2Command(&arm, &elevator, &superStructure));
    oprtr.B().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.X().WhileTrue(L3Command(&arm, &elevator, &superStructure));
    oprtr.X().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.Y().WhileTrue(L4Command(&arm, &elevator, &superStructure));
    oprtr.Y().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    (!driver.A() && oprtr.RightTrigger()).WhileTrue(
            SourceCommand(&arm, &elevator, &intake, &superStructure, &oprtr).BeforeStarting([this] {
                return arm.changeBlockedWrist(100);
            }).AndThen([this] {
                return arm.changeBlockedWrist(124);
            }));
    oprtr.RightTrigger().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    //oprtr.LeftTrigger().WhileTrue(stationPos(&chassis, &tagLayout));

    oprtr.POVDown().WhileTrue(LowAlgae(&arm, &elevator, &intake, &superStructure));
    oprtr.POVDown().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.POVUp().WhileTrue(HighAlgae(&arm, &elevator, &intake, &superStructure));
    oprtr.POVUp().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.LeftBumper().WhileTrue(Processor(&arm, &elevator, &superStructure));
    oprtr.LeftBumper().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    oprtr.Start().WhileTrue(climber.setClimberCommand(ClimberConstants::OpenPosition));
    oprtr.Start().OnFalse(climber.setClimberCommand(ClimberConstants::ClosedPosition));

    oprtr.POVRight().WhileTrue(frc2::cmd::RunOnce([this] {
        armOffset += 1.0_deg;
        arm.updateOffset(armOffset);
    }));
    oprtr.POVLeft().WhileTrue(frc2::cmd::RunOnce([this] {
        armOffset -= 1.0_deg;
        arm.updateOffset(armOffset);
    }));

    increaseOffsetX.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::setModifyXTarget(0.02);
        frc::SmartDashboard::PutBoolean("IncreaseOffset/IncreaseOffsetX", false);
    }));

    decreaseOffsetX.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::setModifyXTarget(-0.02);
        frc::SmartDashboard::PutBoolean("DecreaseOffset/DecreaseOffsetX", false);
    }));

    increaseOffsetLeft.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::setModifyLeftTarget(0.03);
        frc::SmartDashboard::PutBoolean("IncreaseOffset/IncreaseOffsetLeft", false);
    }));

    decreaseOffsetLeft.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::setModifyLeftTarget(-0.03);
        frc::SmartDashboard::PutBoolean("DecreaseOffset/DecreaseOffsetLeft", false);
    }));

    increaseOffsetRight.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::setModifyRightTarget(0.03);
        frc::SmartDashboard::PutBoolean("IncreaseOffset/IncreaseOffsetRight", false);
    }));

    decreaseOffsetRight.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::setModifyRightTarget(-0.03);
        frc::SmartDashboard::PutBoolean("DecreaseOffset/DecreaseOffsetRight", false);
    }));

    resetOffsets.OnTrue(frc2::cmd::RunOnce([this] {
        AlignSpeedHelper::resetOffset();
        frc::SmartDashboard::PutBoolean("ResetOffset", false);
    }));

}

double RobotContainer::getLeftStickDistance() {
    frc::Translation2d joystickPos {units::meter_t(driver.GetLeftX()), units::meter_t(driver.GetLeftY())};
    return std::abs(joystickPos.Distance( {}).value());
}

bool RobotContainer::getDriverOverride() {
    return getLeftStickDistance() > 0.3;
}

void RobotContainer::ConfigMixedBindigs() {
    //NextButton 6

    console.Button(6).OnTrue(frc2::cmd::RunOnce([this] {
        climber.setOffset();
    }));

    (!driver.A() && console.Button(3)).OnTrue(L1Command(&arm, &elevator, &superStructure));
    console.Button(3).OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    (console.Button(12) && driver.POVRight()).OnTrue(
            L2Command(&arm, &elevator, &superStructure).AlongWith(
                    leftAlignPos(&chassis, &tagLayout, &driver).BeforeStarting([this] {
                        disableBackCamera();
                    }).FinallyDo([this] {
                        enableBackCamera();
                    })));
    (console.Button(5) && driver.POVRight()).OnTrue(
            L2Command(&arm, &elevator, &superStructure).AlongWith(
                    rightAlignPos(&chassis, &tagLayout, &driver).BeforeStarting([this] {
                        disableBackCamera();
                    }).FinallyDo([this] {
                        enableBackCamera();
                    })));
    ;

    (console.Button(7) && driver.POVRight()).OnTrue(
            L3Command(&arm, &elevator, &superStructure).AlongWith(
                    leftAlignPos(&chassis, &tagLayout, &driver).BeforeStarting([this] {
                        disableBackCamera();
                    }).FinallyDo([this] {
                        enableBackCamera();
                    })));
    (console.Button(8) && driver.POVRight()).OnTrue(
            L3Command(&arm, &elevator, &superStructure).AlongWith(
                    rightAlignPos(&chassis, &tagLayout, &driver).BeforeStarting([this] {
                        disableBackCamera();
                    }).FinallyDo([this] {
                        enableBackCamera();
                    })));
    ;

    (console.Button(10) && driver.POVRight()).OnTrue(
            L4Command(&arm, &elevator, &superStructure).AlongWith(
                    leftAlignPos(&chassis, &tagLayout, &driver).BeforeStarting([this] {
                        disableBackCamera();
                    }).FinallyDo([this] {
                        enableBackCamera();
                    })));
    (console.Button(11) && driver.POVRight()).OnTrue(
            L4Command(&arm, &elevator, &superStructure).AlongWith(
                    rightAlignPos(&chassis, &tagLayout, &driver).BeforeStarting([this] {
                        disableBackCamera();
                    }).FinallyDo([this] {
                        enableBackCamera();
                    })));
    ;

    (!driver.A() && console.Button(1)).OnTrue(LowAlgae(&arm, &elevator, &intake, &superStructure));
    console.Button(1).OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    (!driver.A() && console.Button(2)).OnTrue(HighAlgae(&arm, &elevator, &intake, &superStructure));
    console.Button(2).OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    (!driver.A() && console.AxisMagnitudeGreaterThan(0, 0.1)).OnTrue(
            SourceCommand(&arm, &elevator, &intake, &superStructure, &console).BeforeStarting([this] {
                return arm.changeBlockedWrist(100);
            }).AndThen([this] {
                return arm.changeBlockedWrist(124);
            }));
    console.AxisMagnitudeGreaterThan(0, 0.1).OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    (!driver.A() && console.Button(9)).OnTrue(Processor(&arm, &elevator, &superStructure));
    console.Button(9).OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    //Align for Processor
    /*.AlongWith(
     processorPos(&chassis, &tagLayout).BeforeStarting([this] {
     //  disableBackCamera();
     }).AndThen([this] {
     //  enableBackCamera();
     }))*/

    (console.Button(4)).OnTrue(climber.setClimberCommand(ClimberConstants::OpenPosition));
    (console.Button(4)).OnFalse(climber.setClimberCommand(ClimberConstants::ClosedPosition));
    driver.POVRight().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));
}

void RobotContainer::changeBlockedWrist() {
    arm.changeBlockedWrist(124);
}

void RobotContainer::ConfigDefaultCommands() {
    //climber.setClimberCommand(ClimberConstants::ClosedPosition);

}

void RobotContainer::ConfigCharacterizationBindings() {

    //test.A().WhileTrue(climber.setClimberCommand(55_deg));
    //test.A().OnFalse(climber.setClimberCommand(-40.0_deg));

    //test.A().WhileTrue(L3Command(&arm, &elevator, &superStructure).AlongWith(leftAlignPos(&chassis, &tagLayout, &driver)));
    //test.A().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    //test.B().WhileTrue(L2Command(&arm, &elevator, &superStructure).AlongWith(rightAlignPos(&chassis, &tagLayout, &driver)));
    //test.B().OnFalse(ClosedCommand(&arm, &elevator, &intake, &superStructure));

    //test.A().ToggleOnTrue(TabulateCommand(&elevator, &arm, &intake).ToPtr());
    //test.A().WhileTrue(intake.setIntakeCommand(0.0_V, 25.0_deg));
    //test.A().OnFalse(intake.setIntakeCommand(0.0_V, 10.0_deg));

    //test.A().WhileTrue(elevator.setElevatorCommand(1_m));
    //test.A().OnFalse(elevator.setElevatorCommand(0.00_m));

    //test.A().WhileTrue(arm.setArmCommand(30_deg, 0_deg));
    //test.A().OnFalse(arm.setArmCommand(90_deg, 0_deg));

    //test.A().WhileTrue(arm.setArmCommand(30_deg, -90_deg));
    //test.A().OnFalse(arm.setArmCommand(30_deg, 0_deg));

    //test.A().WhileTrue(arm.setArmCommand(130_deg, 90_deg));
    //test.A().OnFalse(arm.setArmCommand(90_deg, 0_deg));
    //test.A().WhileTrue(arm.setArmCommand(130_deg, 0_deg));
    //test.A().OnFalse(arm.setArmCommand(90_deg, 0_deg));
    //test.POVDown().WhileTrue(arm.setArmCommand(40_deg, 0_deg));
    //test.POVDown().OnFalse(arm.setArmCommand(40_deg, 0_deg));

    //test.A().WhileTrue(intake.moveIntake(4_V));
    //test.A().OnFalse(intake.moveIntake(0_V));

}

void RobotContainer::disableBackCamera() {
    backLeftCam.setEnabled(false);
}

void RobotContainer::enableBackCamera() {
    backLeftCam.setEnabled(true);
}

AprilTags::Config RobotContainer::frontRightCamera() {
    AprilTags::Config config;
    config.cameraName = "Global_Shutter_Camera";
    config.cameraToRobot = {6.195169_in, -6.064487_in, 6.248962_in, {0_deg, -28.0_deg, 45_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::frontLeftCamera() {
    AprilTags::Config config;
    config.cameraName = "FrontLeft";
    config.cameraToRobot = {9.875_in, 10.653063_in, 8.109802_in, {0_deg, -15_deg, 0_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::backRightCamera() {
    AprilTags::Config config;
    config.cameraName = "BackRight";
    config.cameraToRobot = {10.784188_in, 2.200000_in, 20.810051_in, {0_deg, 15_deg, 0_deg}};
    return config;
}

AprilTags::Config RobotContainer::backLeftCamera() {
    AprilTags::Config config;
    config.cameraName = "BackLeft (1)";
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
