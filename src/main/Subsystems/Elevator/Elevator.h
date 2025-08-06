// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "Subsystems/Elevator/ElevatorConstants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>
#include <frc2/command/FunctionalCommand.h>

class Elevator: public frc2::SubsystemBase {
public:

    Elevator();
    void Periodic() override;

    void setTarget(units::meter_t position); //Provide a target
    void setElevatorUpperSpeed(); //Configurations for when going up
    void setElevatorLowerSpeed(); //Configurations for when going down

    units::meter_t getPosition(); //Ask for the current position

    bool isElevatorAtPosition(units::meter_t elevatorPosition); //Check if it is already at the target position
    frc2::CommandPtr setElevatorCommand(units::meter_t elevatorPosition); //Command that moves the elevator

private:
    //Declaration of the motors involved in this subsystem
    OverTalonFX leftElevatorMotor {ElevatorConstants::LeftConfig(), "rio"};
    OverTalonFX rightElevatorMotor {ElevatorConstants::RightConfig(), "rio"};

    MotionMagicVoltage elevatorVoltage {0_tr};
};
