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
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Subsystems/Intake/Constants.h"

class Intake: public frc2::SubsystemBase {
public:
    Intake();

    void setVoltage(units::volt_t voltage);
    double getVoltage();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    frc2::CommandPtr moveIntake(units::volt_t voltage);

private:
    //ID 20
    OverTalonFX intakeMotor
    { Constants::IntakeConfig(), "rio" };
};
