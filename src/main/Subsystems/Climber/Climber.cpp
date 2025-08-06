// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Climber/Climber.h"

Climber::Climber() {
    climberMotor.SetPosition(0_tr);
    climberMotor.setSensorToMechanism(ClimberConstants::ClimberSensorToMechanism);
    climberMotor.configureMotionMagic(ClimberConstants::ClimberCruiseVelocity,
            ClimberConstants::ClimberCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Climber::setToAngle(units::degree_t climberAngle) {
    frc::SmartDashboard::PutNumber("Climber/TargetArmAngle", climberAngle.value());
    climberMotor.SetControl(climberVoltage.WithPosition(climberAngle).WithEnableFOC(true));

}

frc::Rotation2d Climber::getCurrentClimberAngle() {
    return units::degree_t((climberEncoder.Get() - ClimberConstants::ClimberEncoderOffset) * 360);
}

bool Climber::isClimberAtPosition(units::degree_t climberAngle) {
    units::degree_t climberError = climberAngle - getCurrentClimberAngle().Degrees();
    return (units::math::abs(climberError) < ClimberConstants::ClimberRangeError);
}

frc2::CommandPtr Climber::setClimberCommand(units::degree_t climberAngle) {
    return frc2::FunctionalCommand([this, climberAngle]() {
        setToAngle(climberAngle);
    }, [this, climberAngle]() {
        setToAngle(climberAngle + offset);
    }, [this](bool interupted) {
        offset = 0_deg;
    }, [this, climberAngle]() {
        return isClimberAtPosition(climberAngle);
    },
    {this}).ToPtr();
}

void Climber::setOffset() {
    offset -= 1_deg;
}

void Climber::Periodic() {
}
