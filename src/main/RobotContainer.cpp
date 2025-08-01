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

}

void RobotContainer::ConfigOperatorBindings() {

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

double RobotContainer::getLeftStickDistance() {
    frc::Translation2d joystickPos {units::meter_t(driver.GetLeftX()), units::meter_t(driver.GetLeftY())};
    return std::abs(joystickPos.Distance( {}).value());
}

bool RobotContainer::getDriverOverride() {
    return getLeftStickDistance() > 0.3;
}

void RobotContainer::ConfigMixedBindigs() {

}

void RobotContainer::ConfigDefaultCommands() {

}

void RobotContainer::ConfigCharacterizationBindings() {

}

AprilTags::Config RobotContainer::frontRightCamera() {
    AprilTags::Config config;
    config.cameraName = "FrontRight";
    config.cameraToRobot = {7.200000_in, -5.892500_in, 6.368259_in, {0_deg, -21.500115_deg, 30.026518_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::frontLeftCamera() {
    AprilTags::Config config;
    config.cameraName = "FrontLeft";
    config.cameraToRobot = {6.000000_in, 11.000000_in, 7.752224_in, {0_deg, -21.000118_deg, 25.025948_deg}};
    config.tagValidDistances = { {1, 3.5_m}, {2, 4.0_m}, {3, 4.0_m}};
    return config;
}

AprilTags::Config RobotContainer::backRightCamera() {
    AprilTags::Config config;
    config.cameraName = "MultiCam";
    config.cameraToRobot = {11.000000_in, -7.000000_in, 9.752224_in, {0_deg, -15.000170_deg, 50.018714_deg}};
    return config;
}

AprilTags::Config RobotContainer::backLeftCamera() {
    AprilTags::Config config;
    config.cameraName = "BackLeft";
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
