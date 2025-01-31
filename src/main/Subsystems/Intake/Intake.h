// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Subsystems/Intake/IntakeConstants.h"
#include <frc2/command/FunctionalCommand.h>
#include <units/math.h>

class Intake: public frc2::SubsystemBase {
public:
    Intake();

    void setMotorVoltage(units::volt_t voltage);
    void setToAngle(units::degree_t jawAngle);
    double getVoltage();
    bool isJawAtPosition(units::degree_t jawAngle);
    frc2::CommandPtr setIntakeCommand(units::volt_t voltage, units::degree_t jawAngle);

    void Periodic() override;
    frc2::CommandPtr moveIntake(units::volt_t voltage);

private:
    //ID 20
    MotionMagicVoltage jawVoltage {0_tr};

    OverTalonFX intakeMotor {IntakeConstants::IntakeConfig(), "rio"};
    OverTalonFX intakeJawMotor {IntakeConstants::IntakeJawConfig(), "rio"};
};
