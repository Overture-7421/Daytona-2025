// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h>
#include <OvertureLib/Utils/Logging/Logging.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <units/math.h>

#include "Subsystems/Intake/IntakeConstants.h"

#include <ctre/phoenix6/CANrange.hpp>

class Intake: public frc2::SubsystemBase {
public:
    Intake();

    void setMotorVoltage(units::volt_t voltage);

    double getVoltage();

    bool isCoralIn();
    bool isAlgaeIn();

    frc2::CommandPtr moveIntake(units::volt_t voltage);

    void Periodic() override;

private:

    VoltageOut intakeVoltage {0_V};

    OverTalonFX intakeMotor {IntakeConstants::IntakeConfig(), "rio"};

    CANrange canRange {30, "rio"};

};
