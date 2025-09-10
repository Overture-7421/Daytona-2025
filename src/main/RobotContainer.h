// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <OvertureLib/Robots/OverContainer/OverContainer.h>
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>
#include <OvertureLib/Gamepads/OverConsole/OverConsole.h>
#include <OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include "Subsystems/Chassis/Chassis.h"
#include "Commands/DriveCommand/DriveCommand.h"
#include "Commands/ResetHeading/ResetHeading.h"
#include "Commands/AlignPositions/AlignPositions.h"

#include "Subsystems/Grabber/Grabber.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Climber/Climber.h"
#include "Subsystems/Intake/Intake.h"
#include "Manager/AlignManager/AlignManager.h"
#include "Manager/StateManager/StateManager.h"

#include "Commands/AlgaeCommands/AlgaeCommands.h"
#include "Commands/ReefCommands/ReefCommands.h"
#include "Commands/SustainedCommands/SustainedCommands.h"
#include "Commands/EndPositionCommands/EndPositionCommands.h"
#include "Commands/ConfirmCommand/ConfirmCommand.h"
#include "Commands/EmergencyCommand/EmergencyCommand.h"
#include "Commands/CharacterizationCommand/CharacterizationCommand.h"
#include "Commands/ClosedCommand/ClosedCommand.h"

class RobotContainer: public OverContainer {
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
    void UpdateTelemetry();

private:

    void ConfigureBindings();
    void ConfigDriverBindings();
    void ConfigOperatorBindings();
    void ConfigMixedBindigs();
    void ConfigDefaultCommands();
    void ConfigCharacterizationBindings();
    void disableBackCamera();
    void enableBackCamera();

    OverXboxController driver {0, 0.65, 0.2};
    OverXboxController oprtr {1, 0.20, 0.2};
    OverConsole console {2};
    OverXboxController test {3, 0.20, 0.2};

#ifndef __FRC_ROBORIO__
    frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
#else
	frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
	//frc::AprilTagFieldLayout tagLayout{ "/home/lvuser/deploy/tag_layout/7421-field.json" };
#endif 
    double getLeftStickDistance();
    bool getDriverOverride();

    //Subsystems
    Chassis chassis;
    Grabber grabber;
    Elevator elevator;
    Arm arm;
    Climber climber;
    Intake intake;

    AlignManager alignManager {&chassis, &tagLayout};
    StateManager stateManager {&intake, &arm, &elevator, &grabber, &climber, &alignManager, &driver, &oprtr, &console,
            &endToInitial};

    static AprilTags::Config railCameraRight();
    static AprilTags::Config climberCameraLeft();
    static AprilTags::Config climberCameraRight();
    static AprilTags::Config railCameraLeft();

    AprilTags railCamRight {&tagLayout, &chassis, railCameraRight()};
    AprilTags climberCamLeft {&tagLayout, &chassis, climberCameraLeft()};
    AprilTags climberCamRight {&tagLayout, &chassis, climberCameraRight()};
    AprilTags railCamLeft {&tagLayout, &chassis, railCameraLeft()};

    frc::SendableChooser<frc2::Command*> autoChooser;

    frc2::Trigger endToInitial {[] {
        return frc::SmartDashboard::GetBoolean("EndToInitial", false);
    }};

    frc2::Trigger emergency {[] {
        return frc::SmartDashboard::GetBoolean("EMERGENCY", false);
    }};

    // frc2::Trigger startCommands{ [this]() {
    // 	return stateManager.getExecute();
    // } };

    // Maybe si lo usamos
    // frc2::Trigger increaseOffsetX {[] {
    //     return frc::SmartDashboard::GetBoolean("IncreaseOffset/IncreaseOffsetX", false);
    // }};
    // frc2::Trigger decreaseOffsetX {[] {
    //     return frc::SmartDashboard::GetBoolean("DecreaseOffset/DecreaseOffsetX", false);
    // }};

    // frc2::Trigger increaseOffsetLeft {[] {
    //     return frc::SmartDashboard::GetBoolean("IncreaseOffset/IncreaseOffsetLeft", false);
    // }};
    // frc2::Trigger decreaseOffsetLeft {[] {
    //     return frc::SmartDashboard::GetBoolean("DecreaseOffset/DecreaseOffsetLeft", false);
    // }};

    // frc2::Trigger increaseOffsetRight {[] {
    //     return frc::SmartDashboard::GetBoolean("IncreaseOffset/IncreaseOffsetRight", false);
    // }};
    // frc2::Trigger decreaseOffsetRight {[] {
    //     return frc::SmartDashboard::GetBoolean("DecreaseOffset/DecreaseOffsetRight", false);
    // }};

    // frc2::Trigger resetOffsets {[] {
    //     return frc::SmartDashboard::GetBoolean("ResetOffset", false);
    // }};

};
