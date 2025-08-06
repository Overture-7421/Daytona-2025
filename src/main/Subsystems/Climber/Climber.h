// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/DutyCycleEncoder.h>

#include "Subsystems/Climber/ClimberConstants.h"

class Climber: public frc2::SubsystemBase {
public:
    Climber();

    void setToAngle(units::degree_t climberAngle);
    frc::Rotation2d getCurrentClimberAngle();
    bool isClimberAtPosition(units::degree_t climberAngle);

    frc2::CommandPtr setClimberCommand(units::degree_t climberAngle);

    void setOffset();
    units::degree_t offset = 0_deg;

    void Periodic() override;

private:

    OverTalonFX climberMotor {ClimberConstants::ClimberConfig(), "rio"};
    frc::DutyCycleEncoder climberEncoder {0}; //Puerto en la RoboRio donde va a estar (No definido aun)

    MotionMagicVoltage climberVoltage {0_tr};
};
