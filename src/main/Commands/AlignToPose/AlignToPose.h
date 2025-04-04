// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Chassis/Chassis.h"
#include "SpeedHelpers/AlignSpeedHelper/AlignSpeedHelper.h"
#include "Enums/ReefSide.h"
#include "Commands/AlignToPose/AlignPositions.h"

class AlignToPose: public frc2::CommandHelper<frc2::Command, AlignToPose> {
public:
    AlignToPose(Chassis *chassis, ReefSide direction, frc::AprilTagFieldLayout *tagLayout);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    Chassis *chassis;
    std::shared_ptr<AlignSpeedHelper> alignSpeedHelper;
    frc::AprilTagFieldLayout *tagLayout;

    ReefOffset reefOffset;
    ReefSide reefSide;

};
