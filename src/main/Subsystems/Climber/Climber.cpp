// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Climber/Climber.h"

Climber::Climber() {
    climberMotor.setSensorToMechanism(ClimberConstants::ClimberSensorToMechanism);
    climberPID.DisableContinuousInput();

}

units::degree_t Climber::getCurrentClimberAngle() {
    return units::degree_t((climberEncoder.Get() - ClimberConstants::ClimberEncoderOffset) * 360);
}

void Climber::setTarget(units::degree_t climberTarget) {
    this->target = climberTarget;
}

bool Climber::isClimberAtPosition(units::degree_t climberAngle) {
    return (units::math::abs(climberAngle - getCurrentClimberAngle()) < ClimberConstants::ClimberRangeError);
}

frc2::CommandPtr Climber::setState(Positions climberState) {
    return frc2::FunctionalCommand([this, climberState]() {
        setTarget(ClimberConstants::ClimberPositions.at(climberState));
    }, [this, climberState]() {
        setTarget(ClimberConstants::ClimberPositions.at(climberState) + offset);
    }, [this](bool interupted) {
        offset = 0.0_deg;
    }, [this, climberState]() {
        return isClimberAtPosition(ClimberConstants::ClimberPositions.at(climberState));
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::setCharacterization(units::degree_t angle) {
    return frc2::FunctionalCommand([this, angle]() {
        setTarget(angle);
    }, [this, angle]() {
        setTarget(angle + offset);
    }, [this](bool interupted) {
        offset = 0.0_deg;
    }, [this, angle]() {
        return isClimberAtPosition(angle);
    },
    {this}).ToPtr();
}

void Climber::setOffset() {
    offset -= 1.0_deg;
}

void Climber::setServoAngle(units::degree_t angle) { //Conditional that allows us to invert the right servo if needed.
    servo.Set(angle.value());
}

frc2::CommandPtr Climber::servoAngleCommand(units::degree_t angle) {
    return frc2::cmd::RunOnce([this, angle] {
        this->setServoAngle(angle);
    });
}

void Climber::Periodic() {

    units::volt_t motorOutput = units::volt_t(climberPID.Calculate(getCurrentClimberAngle(), target));
    climberMotor.SetControl(climberVoltage.WithOutput(motorOutput).WithEnableFOC(true));

    frc::SmartDashboard::PutNumber("Climber/CurrentThroughbore", getCurrentClimberAngle().value());
    frc::SmartDashboard::PutNumber("Climber/Without360", climberEncoder.Get() - ClimberConstants::ClimberEncoderOffset);
    frc::SmartDashboard::PutNumber("Climber/MotorOutput", motorOutput.value());
    frc::SmartDashboard::PutNumber("Climber/Target", target.value());

}
