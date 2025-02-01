// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
    intakeJawMotor.setRotorToSensorRatio(IntakeConstants::RotorToSensor);
    intakeJawMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity,
            IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Intake::setToAngle(units::degree_t jawAngle) {
    frc::SmartDashboard::PutNumber("IntakeTarget/JawTargetPosition", jawAngle.value());
    intakeJawMotor.SetControl(jawVoltage.WithPosition(jawAngle).WithEnableFOC(true));
}

void Intake::setMotorVoltage(units::volt_t voltage) {
    frc::SmartDashboard::PutNumber("IntakeTarget/IntakeTargetVoltage", voltage.value());
    intakeMotor.SetVoltage(voltage);
}

bool Intake::isJawAtPosition(units::degree_t jawAngle) {
    units::degree_t jawError = jawAngle - intakeJawMotor.GetPosition().GetValue();
    return (units::math::abs(jawError) < 1.0_deg);
}

double Intake::getVoltage() {
    return intakeMotor.GetMotorVoltage().GetValueAsDouble();
}

bool Intake::isCoralIn(units::degree_t jawAngle) {
    return ((canRange.GetDistance(0.30).GetValue() == 0.30_m) && (isJawAtPosition(jawAngle)));
}

bool Intake::isAlgaeIn(units::degree_t jawAngle) {
    return ((canRange.GetDistance(0.10).GetValue() == 0.10_m) && (isJawAtPosition(jawAngle)));
}

frc2::CommandPtr Intake::setIntakeCommand(units::volt_t voltage, units::degree_t jawAngle) {
    return frc2::FunctionalCommand([this, voltage, jawAngle]() {
        setToAngle (jawAngle), setMotorVoltage(voltage);
    }, []() {
    }, [](bool interupted) {
    }, [this, voltage, jawAngle]() {
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

frc2::CommandPtr Intake::setState(IntakeStates state) {
    return this->RunOnce([this, state] {
        this->state = state;
    });
}

IntakeStates Intake::getState() {
    return state;
}

void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Intake/ACTIVATED?", getVoltage() > 0.0);
    double jawCurrentAngle = intakeJawMotor.GetPosition().GetValueAsDouble() * 360;
    frc::SmartDashboard::PutNumber("Intake/CurrentJawAngle", jawCurrentAngle);

}
