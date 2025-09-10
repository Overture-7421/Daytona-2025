// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Servo.h>

#include "Subsystems/Climber/ClimberConstants.h"

class Climber: public frc2::SubsystemBase {
public:
    Climber();

    units::degree_t getCurrentClimberAngle();
    void setTarget(units::degree_t climberTarget);
    bool isClimberAtPosition(units::degree_t climberAngle);

    frc2::CommandPtr setState(Positions state);
    frc2::CommandPtr setCharacterization(units::degree_t angle);

    void setOffset();

    void setServoAngle(units::degree_t angle);
    frc2::CommandPtr servoAngleCommand(units::degree_t angle);

    void Periodic() override;

private:

    OverTalonFX climberMotor {ClimberConstants::ClimberConfig(), "rio"};
    frc::DutyCycleEncoder climberEncoder {8}; //Puerto en la RoboRio donde va a estar (No definido aun)

    VoltageOut climberVoltage {0_V};

    units::degree_t offset = 0.0_deg;
    units::degree_t target = 233_deg; //aquí se pone la posición inicial (NO definido aun)

    frc::ProfiledPIDController<units::degree> climberPID {1.7, 0.0, 0.0, {ClimberConstants::ClimberVelocity,
            ClimberConstants::ClimberAcceleration}};

    frc::Servo servo {9}; // Not defined yet

};
