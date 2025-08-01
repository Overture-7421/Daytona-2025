#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/FunctionalCommand.h>
#include <OvertureLib/Utils/Logging/Logging.h>

#include "Subsystems/Arm/ArmConstants.h"

class Arm: public frc2::SubsystemBase {
public:
    Arm();

    void Periodic() override;

private:

};
