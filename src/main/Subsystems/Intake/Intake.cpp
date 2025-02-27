// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
    intakeJawMotor.setSensorToMechanism(IntakeConstants::SensorToMechanism);
    intakeJawMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity,
            IntakeConstants::IntakeCruiseAcceleration, 0.0_tr_per_s_cu);
    intakeJawMotor.SetPosition(0_tr);
}

void Intake::setToAngle(units::degree_t jawAngle) {
    frc::SmartDashboard::PutNumber("Intake/JawTargetPosition", jawAngle.value());
    intakeJawMotor.SetControl(jawVoltage.WithPosition(jawAngle).WithEnableFOC(true));
}

void Intake::setMotorVoltage(units::volt_t voltage) {
    frc::SmartDashboard::PutNumber("Intake/IntakeTargetVoltage", voltage.value());
    intakeMotor.SetControl(intakeVoltage.WithOutput(voltage).WithEnableFOC(true));
}

bool Intake::isJawAtPosition(units::degree_t jawAngle) {
    units::degree_t jawError = jawAngle - intakeJawMotor.GetPosition().GetValue();
    frc::SmartDashboard::PutNumber("JawError/JawError", jawError.value());
    return (units::math::abs(jawError) < IntakeConstants::RangeError);
}

double Intake::getVoltage() {
    return intakeMotor.GetMotorVoltage().GetValueAsDouble();
}

units::degree_t Intake::getJawAngle() {
    return intakeJawMotor.GetPosition().GetValue();
}

bool Intake::isCoralIn(units::degree_t jawAngle) {
    // return ((canRange.GetDistance().GetValue() <= IntakeConstants::SensorCoralDistance));

    return canRange.GetIsDetected().GetValue();
}

bool Intake::isAlgaeIn(units::degree_t jawAngle) {
    //return ((canRange.GetDistance().GetValue() <= IntakeConstants::SensorAlgaeDistance));

    return canRange.GetIsDetected().GetValue();

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

    units::degree_t jawCurrentAngleMotor = intakeJawMotor.GetPosition().GetValue();
    frc::SmartDashboard::PutNumber("IntakeCurrent/CurrentJawAngleMotor", jawCurrentAngleMotor.value());
    frc::SmartDashboard::PutNumber("IntakeCurrent/Voltage", intakeJawMotor.GetMotorVoltage().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("IntakeCurrent/Curent-Amps", intakeJawMotor.GetSupplyCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutBoolean("Sensor Activated???", isCoralIn(10_deg));
    frc::SmartDashboard::PutNumber("SensorDistance", canRange.GetDistance().GetValue().value());

}
