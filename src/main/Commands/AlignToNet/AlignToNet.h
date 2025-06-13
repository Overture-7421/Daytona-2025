// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/Chassis/Chassis.h"
#include "SpeedHelpers/AlignNetSpeedHelper/AlignNetSpeedHelper.h"

class AlignToNet: public frc2::CommandHelper<frc2::Command, AlignToNet> {
public:

    AlignToNet(Chassis *chassis, frc::Pose2d pose2d);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    Chassis *chassis;
    AlignNetSpeedHelper alignNetSpeedHelper;
};
