#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <OvertureLib/Utils/Logging/Logging.h>
#include "Enums/Heading.h"
#include <frc2/command/Commands.h>

#include "Subsystems/Arm/ArmConstants.h"

class Arm: public frc2::SubsystemBase {
public:
    Arm();

    void setToAngle(units::degree_t armAngle);
    bool isArmAtPosition(units::degree_t armAngle);
    units::degree_t getCurrentAngle();
    frc2::CommandPtr setState(Positions state, Heading heading);
    frc2::CommandPtr setState(Positions state);
    frc2::CommandPtr setCharacterization(units::degree_t angle);
    frc2::CommandPtr setArmZero();

    void Periodic() override;

private:

    MotionMagicVoltage armVoltage {0_tr};
    OverTalonFX armMotor {ArmConstants::ArmConfig(), "rio"};
    OverCANCoder armCANCoder {ArmConstants::ArmCANConfig(), "rio"};
};
