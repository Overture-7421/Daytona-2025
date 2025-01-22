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

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void setPosition(units::meter_t position);

    bool isElevatorAtPosition(units::meter_t elevatorPosition);
    frc2::CommandPtr setElevatorCommand(units::meter_t elevatorPosition);

private:
    OverTalonFX leftElevatorMotor {ElevatorConstants::LeftConfig(), "rio"};
    OverTalonFX rightElevatorMotor {ElevatorConstants::RightConfig(), "rio"};

    MotionMagicVoltage elevatorVoltage {0_tr};

    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};
