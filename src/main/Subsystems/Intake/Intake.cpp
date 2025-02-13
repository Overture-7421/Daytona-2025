// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
    intakeJawMotor.setSensorToMechanism(IntakeConstants::SensorToMechanism);
    intakeJawMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity,
            IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Intake::setToAngle(units::degree_t jawAngle) {
    frc::SmartDashboard::PutNumber("Intake/JawTargetPosition", jawAngle.value());
    intakeJawMotor.SetControl(jawVoltage.WithPosition(jawAngle).WithEnableFOC(true));
}

void Intake::setMotorVoltage(units::volt_t voltage) {
    frc::SmartDashboard::PutNumber("Intake/IntakeTargetVoltage", voltage.value());
    intakeMotor.SetVoltage(voltage);
}

bool Intake::isJawAtPosition(units::degree_t jawAngle) {
    units::degree_t jawError = jawAngle - intakeJawMotor.GetPosition().GetValue();
    frc::SmartDashboard::PutNumber("JawError/JawError", jawError.value());
    return (units::math::abs(jawError) < IntakeConstants::RangeError);
}

double Intake::getVoltage() {
    return intakeMotor.GetMotorVoltage().GetValueAsDouble();
}

bool Intake::isCoralIn(units::degree_t jawAngle) {
    return ((canRange.GetDistance().GetValue() == 0.30_m) && (isJawAtPosition(jawAngle)));
}

bool Intake::isAlgaeIn(units::degree_t jawAngle) {
    return ((canRange.GetDistance().GetValue() == 0.10_m) && (isJawAtPosition(jawAngle)));
}

frc2::CommandPtr Intake::setIntakeCommand(units::volt_t voltage, units::degree_t jawAngle) {
    return frc2::FunctionalCommand([this, voltage, jawAngle]() {
        setToAngle (jawAngle), setMotorVoltage(voltage);
    }, []() {
    }, [](bool interupted) {
    }, [this, jawAngle]() {
        frc::SmartDashboard::PutBoolean("JawError/isAtPosition", isJawAtPosition(jawAngle));
        return isJawAtPosition(jawAngle);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Intake::setJawCommand(units::degree_t jawAngle) {
    return frc2::FunctionalCommand([this, jawAngle]() {
        setToAngle(jawAngle);
    }, []() {
    }, [](bool interupted) {
    }, [this, jawAngle]() {
        return isJawAtPosition(jawAngle);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Intake::moveIntake(units::volt_t voltage) {
    return this->RunOnce([this, voltage] {
        this->setMotorVoltage(voltage);
    });
}

void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Intake/ACTIVATED?", getVoltage() > 0.0);
    double jawCurrentAngle = intakeJawMotor.GetPosition().GetValueAsDouble() * 360;
    frc::SmartDashboard::PutNumber("Intake/CurrentJawAngle", jawCurrentAngle);

}
