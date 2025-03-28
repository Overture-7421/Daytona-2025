// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
}

void Intake::setMotorVoltage(units::volt_t voltage) {
    frc::SmartDashboard::PutNumber("Intake/IntakeTargetVoltage", voltage.value());
    intakeMotor.SetControl(intakeVoltage.WithOutput(voltage).WithEnableFOC(true));
}

double Intake::getVoltage() {
    return intakeMotor.GetMotorVoltage().GetValueAsDouble();
}

bool Intake::isCoralIn() {

    return canRange.GetIsDetected().GetValue();
}

bool Intake::isAlgaeIn() {

    return canRange.GetIsDetected().GetValue();

    //return units::math::abs(intakeMotor.GetSupplyCurrent().GetValue()) > 31.0_A;

}

frc2::CommandPtr Intake::moveIntake(units::volt_t voltage) {
    return this->RunOnce([this, voltage] {
        this->setMotorVoltage(voltage);
    });
}

void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Intake/ACTIVATED?", getVoltage() > 0.0);
    frc::SmartDashboard::PutBoolean("Sensor Activated???", isCoralIn());
    frc::SmartDashboard::PutNumber("SensorDistance", canRange.GetDistance().GetValue().value());
}
