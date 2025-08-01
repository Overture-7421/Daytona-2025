#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Subsystems/SuperStructure/SuperStructureStates.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>

class SuperStructure: public frc2::SubsystemBase {
public:
    SuperStructure();

    void Periodic() override;

private:

};
