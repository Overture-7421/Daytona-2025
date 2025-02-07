// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Subsystems/SuperStructure/SuperStructureStates.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>

class SuperStructure: public frc2::SubsystemBase {
public:
    SuperStructure();

    void Periodic() override;

    frc2::CommandPtr setState(SuperStructureStates state);
    SuperStructureStates getState();

private:

    SuperStructureStates state = SuperStructureStates::HoldCoral;

};
