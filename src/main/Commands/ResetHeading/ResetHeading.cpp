// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ResetHeading.h"
#include <OvertureLib/Utils/UtilityFunctions/UtilityFunctions.h>

frc2::CommandPtr ResetHeading(Chassis *chassis) {
    if (isRedAlliance()) {
        return frc2::cmd::RunOnce([chassis] {
            return chassis->resetHeading(180.0_deg);
        });
    } else {
        return frc2::cmd::RunOnce([chassis] {
            return chassis->resetHeading(0.0_deg);
        });
    }
}
