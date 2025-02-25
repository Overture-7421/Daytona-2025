// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Intake/Intake.h"

class TabulateCommand: public frc2::CommandHelper<frc2::Command, TabulateCommand> {
public:

    TabulateCommand(Elevator *elevator, Arm *arm, Intake *intake);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:

    Elevator *elevator;
    Arm *arm;
    Intake *intake;

};
