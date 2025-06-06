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

#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Climber/Climber.h"
#include "Subsystems/SuperStructure/SuperStructure.h"
#include "Commands/ReefCommands/ReefCommands.h"
#include "Commands/CoralGroundGrabCommand/CoralGroundGrabCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/LowAlgae/LowAlgae.h"
#include "Commands/HighAlgae/HighAlgae.h"
#include "Commands/SourceCommand/SourceCommand.h"
#include "Commands/Processor/Processor.h"
#include "Commands/AlignToPose/AlignToPose.h"
#include "Commands/AlignToNet/AlignToNet.h"
#include "Commands/NetCommand/NetCommand.h"
#include "Commands/AlignToPose/AlignPositions.h"
#include "Commands/AlgaeGroundGrabCommand/AlgaeGroundGrabCommand.h"
#include "Commands/SpitGamePiece/SpitGamePiece.h"

class RobotContainer: public OverContainer {
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
    void UpdateTelemetry();
    void changeBlockedWrist();

private:

    void ConfigureBindings();
    void ConfigDriverBindings();
    void ConfigOperatorBindings();
    void ConfigMixedBindigs();
    void ConfigDefaultCommands();
    void ConfigCharacterizationBindings();
    void disableBackCamera();
    void enableBackCamera();

    OverXboxController driver {0, 0.20, 0.2};
    OverXboxController oprtr {1, 0.20, 0.2};
    OverConsole console {2};
    //OverXboxController test {3, 0.20, 0.2};

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
    Intake intake;
    Elevator elevator;
    Arm arm;
    Climber climber;
    SuperStructure superStructure;

    units::degree_t armOffset = 0_deg;

    static AprilTags::Config frontRightCamera();
    static AprilTags::Config frontLeftCamera();
    static AprilTags::Config backRightCamera();
    static AprilTags::Config backLeftCamera();

    AprilTags frontRightCam {&tagLayout, &chassis, frontRightCamera()};
    AprilTags frontLeftCam {&tagLayout, &chassis, frontLeftCamera()};
    AprilTags backRightCam {&tagLayout, &chassis, backRightCamera()};
    AprilTags backLeftCam {&tagLayout, &chassis, backLeftCamera()};

    frc::SendableChooser<frc2::Command*> autoChooser;

    frc2::Trigger increaseOffsetX {[] {
        return frc::SmartDashboard::GetBoolean("IncreaseOffset/IncreaseOffsetX", false);
    }};
    frc2::Trigger decreaseOffsetX {[] {
        return frc::SmartDashboard::GetBoolean("DecreaseOffset/DecreaseOffsetX", false);
    }};

    frc2::Trigger increaseOffsetLeft {[] {
        return frc::SmartDashboard::GetBoolean("IncreaseOffset/IncreaseOffsetLeft", false);
    }};
    frc2::Trigger decreaseOffsetLeft {[] {
        return frc::SmartDashboard::GetBoolean("DecreaseOffset/DecreaseOffsetLeft", false);
    }};

    frc2::Trigger increaseOffsetRight {[] {
        return frc::SmartDashboard::GetBoolean("IncreaseOffset/IncreaseOffsetRight", false);
    }};
    frc2::Trigger decreaseOffsetRight {[] {
        return frc::SmartDashboard::GetBoolean("DecreaseOffset/DecreaseOffsetRight", false);
    }};

    frc2::Trigger resetOffsets {[] {
        return frc::SmartDashboard::GetBoolean("ResetOffset", false);
    }};

};
