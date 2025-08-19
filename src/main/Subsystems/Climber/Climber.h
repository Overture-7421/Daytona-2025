// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>

#include "Subsystems/Climber/ClimberConstants.h"

class Climber: public frc2::SubsystemBase {
public:
    Climber();

    void setVoltage(units::volt_t climberVoltage);
    frc::Rotation2d getCurrentClimberAngle();
    void setTarget(double climberTarget);
    bool isClimberAtPosition(double climberAngle);

    frc2::CommandPtr setState(Positions state);

    void setOffset();

    void Periodic() override;

private:

    OverTalonFX climberMotor {ClimberConstants::ClimberConfig(), "rio"};
    frc::DutyCycleEncoder climberEncoder {0}; //Puerto en la RoboRio donde va a estar (No definido aun)

    MotionMagicVoltage climberVoltage {0_tr};

    double offset = 0.0;
    double target = 0.0; //aquí se pone la posición inicial

    frc::PIDController climberPID {0.0, 0.0, 0.0};

};
