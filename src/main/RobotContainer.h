// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <OvertureLib/Robots/OverContainer/OverContainer.h>
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>
#include <OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h>
#include "Subsystems/Chassis/Chassis.h"
#include "Commands/DriveCommand/DriveCommand.h"
#include "Commands/ResetHeading/ResetHeading.h"

#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Climber/Climber.h"

#include "Commands/L1Command/L1Command.h"
#include "Commands/L2Command/L2Command.h"
#include "Commands/L3Command/L3Command.h"
#include "Commands/L4Command/L4Command.h"
#include "Commands/CoralGroundGrabCommand/CoralGroundGrabCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"
#include "Commands/ClosedCommand/ClosedCommand.h"
#include "Commands/LowAlgae/LowAlgae.h"
#include "Commands/HighAlgae/HighAlgae.h"
#include "Commands/SourceCommand/SourceCommand.h"
#include "Commands/Processor/Processor.h"
#include "Commands/AlignToPose/AlignToPose.h"
#include "Commands/NetCommand/NetCommand.h"
#include "Commands/AlignToPose/AlignPositions.h"
#include "Commands/AlgaeGroundGrabCommand/AlgaeGroundGrabCommand.h"

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

    OverXboxController driver {0, 0.20, 0.2};
    OverXboxController oprtr {1, 0.20, 0.2};

#ifndef __FRC_ROBORIO__
    frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
#else
    frc::AprilTagFieldLayout tagLayout{ "/home/lvuser/deploy/tag_layout/7421-field.json" };

#endif 
    //Subsystems
    Chassis chassis;
    Intake intake;
    Elevator elevator;
    Arm arm;
    Climber climber;

    static AprilTags::Config frontRightCamera();
    static AprilTags::Config frontLeftCamera();
    static AprilTags::Config backRightCamera();
    static AprilTags::Config backLeftCamera();

    AprilTags frontRightCam {&tagLayout, &chassis, frontRightCamera()};
    AprilTags frontLeftCam {&tagLayout, &chassis, frontLeftCamera()};
    AprilTags backRightCam {&tagLayout, &chassis, backRightCamera()};
    AprilTags backLeftCam {&tagLayout, &chassis, backLeftCamera()};

    frc::SendableChooser<frc2::Command*> autoChooser;

};
