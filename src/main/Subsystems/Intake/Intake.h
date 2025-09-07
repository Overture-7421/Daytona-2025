// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <ctre/phoenix6/CANrange.hpp>

#include "Subsystems/Intake/IntakeConstants.h"

class Intake: public frc2::SubsystemBase {
public:
    Intake();

    void setIntakeToAngle(units::degree_t intakeAngle);
    bool isIntakeAtPosition(units::degree_t intakeAngle);

    void setRollersVoltage(units::volt_t voltage);
    void setCenteringVoltage(units::volt_t voltage);

    bool isCoralIn();

    units::degree_t getIntakePosition();

    frc2::CommandPtr setState(Positions state);
    frc2::CommandPtr setCharacterization(units::volt_t rollers, units::volt_t centering, units::degree_t intake);

    void Periodic() override;

private:

    OverTalonFX intakeMotor {IntakeConstants::IntakeConfig(), "rio"};
    OverCANCoder intakeCANCoder {IntakeConstants::IntakeCANConfig(), "rio"};

    OverTalonFX rollersMotor {IntakeConstants::RollersConfig(), "rio"};
    OverTalonFX centeringMotor {IntakeConstants::CenteringConfig(), "rio"};

    CANrange canRange {30, "rio"};

    MotionMagicVoltage intakeVoltage {0_tr};
    VoltageOut rollersVoltage {0_V};
    VoltageOut centeringVoltage {0_V};

};
