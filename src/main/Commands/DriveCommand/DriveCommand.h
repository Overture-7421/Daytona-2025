// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/Chassis/Chassis.h"

#include <frc/controller/ProfiledPIDController.h>
#include <OvertureLib/Math/TargetingWhileMoving/TargetingWhileMoving.h>

#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveCommand: public frc2::CommandHelper<frc2::Command, DriveCommand> {
public:
    DriveCommand(Chassis *chassis, OverXboxController *gamepad);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:

    Chassis *chassis;
    OverXboxController *gamepad;

    frc::ProfiledPIDController<units::radian> headingController {
    // PID constants: 
            5, 0.0, 0.0, frc::TrapezoidProfile < units::radian > ::Constraints {1000_deg_per_s, 850_deg_per_s / 1_s} //Constraints max velocity, max acceleration
    };
    HeadingSpeedsHelper headingSpeedsHelper;

    frc::Translation2d targetObjective;

    frc::SlewRateLimiter<units::meters_per_second> xInput {18_mps_sq};
    frc::SlewRateLimiter<units::meters_per_second> yInput {18_mps_sq};

    int allianceMulti;
    double slowMulti = 1;

    bool speedHelperMoved = false;
};
