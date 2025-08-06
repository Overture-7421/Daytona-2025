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
#include "Subsystems/Grabber/GrabberConstants.h"
#include <ctre/phoenix6/configs/Configs.hpp>

class Grabber: public frc2::SubsystemBase {
public:
    Grabber();

    void setMotorVoltage(units::volt_t voltage); //Provides vltage to the motor
    double getVoltage(); //Retrieves the current voltage

    bool isCoralIn(); //Checks if the grabber is holding a Coral
    bool isAlgaeIn(); //Checks if the grabber is holding an Algae

    frc2::CommandPtr moveGrabber(units::volt_t voltage); //simply applies voltage

    void Periodic() override;

private:

    VoltageOut grabberVoltage {0_V};

    OverTalonFX grabberMotor {GrabberConstants::GrabberConfig(), "rio"};

};
