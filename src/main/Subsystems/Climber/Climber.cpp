// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Climber/Climber.h"

Climber::Climber() {
    climberMotor.setSensorToMechanism(ClimberConstants::ClimberSensorToMechanism);
    climberMotor.configureMotionMagic(ClimberConstants::ClimberCruiseVelocity,
            ClimberConstants::ClimberCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Climber::setVoltage(units::volt_t voltage) {
    climberMotor.SetControl(climberVoltage.with);

}

frc::Rotation2d Climber::getCurrentClimberAngle() {
    return units::degree_t((climberEncoder.Get() - ClimberConstants::ClimberEncoderOffset) * 360);
}

void Climber::setTarget(double climberTarget) {
    this->target = climberTarget;
}

bool Climber::isClimberAtPosition(double climberAngle) {
    double climberError = climberAngle - getCurrentClimberAngle().m_value.value();
    return (units::math::abs(climberError) < ClimberConstants::ClimberRangeError);
}

frc2::CommandPtr Climber::setState(Positions climberState) {
    return frc2::FunctionalCommand([this, climberState]() {
        setTarget(ClimberConstants::ClimberPositions.at(climberState));
    }, [this, climberState]() {
        setTarget(climberState + offset);
    }, [this](bool interupted) {
        offset = 0_deg;
    }, [this, climberAngle]() {
        return isClimberAtPosition(ClimberConstants::ClimberPositions.at(climberState));
    },
    {this}).ToPtr();
}

void Climber::setOffset() {
    offset -= 1_deg;
}

void Climber::Periodic() {

    double motorOutput = climberPID.calculate(getCurrentClimberAngle(), target);
    climberMotor.SetVoltage(motorOutput);
}
