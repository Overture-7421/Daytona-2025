// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake(){
    intakeJawMotor.setRotorToSensorRatio(IntakeConstants::RotorToSensor);

    intakeJawMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration,
            0.0_tr_per_s_cu);
}

void Intake::setToAngle(units::volt_t voltage, units::degree_t jawAngle) {
    frc::SmartDashboard::PutNumber("JawTarget/JawTargetPosition", jawAngle.value());
    frc::SmartDashboard::PutNumber("IntakeTarget/IntakeTargetVoltage", voltage.value());

    intakeMotor.SetVoltage(voltage);
    intakeJawMotor.SetControl(jawVoltage.WithPosition(jawAngle).WithEnableFOC(true));

}

bool Intake::isJawAtPosition(units::degree_t jawAngle) {
    units::degree_t armError = jawAngle - intakeJawMotor.GetPosition().GetValue();

    return (armError.value() < 0.5);
}

frc2::CommandPtr Intake::setIntakeCommand(units::volt_t voltage, units::degree_t jawAngle) {
    return frc2::FunctionalCommand([this, voltage, jawAngle]() {
        setToAngle(voltage, jawAngle);
    }, []() {
    }, [](bool interupted) {
    }, [this, voltage, jawAngle]() {
        return isJawAtPosition(jawAngle);
    },
    {this}).ToPtr();
}

void Intake::setMotorVoltage(units::volt_t voltage) {

    intakeMotor.SetVoltage(voltage);
}

double Intake::getVoltage() {
    return intakeMotor.GetMotorVoltage().GetValueAsDouble();
}

frc2::CommandPtr Intake::moveIntake(units::volt_t voltage) {
    return this->RunOnce([this, voltage] {
        this->setMotorVoltage(voltage);
    });
}
;

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Intake/ACTIVATED?", getVoltage() > 0.0);
    double jawCurrentAngle = intakeJawMotor.GetPosition().GetValueAsDouble() * 360;
    frc::SmartDashboard::PutNumber("Intake/Current Jaw Angle", jawCurrentAngle);

}
