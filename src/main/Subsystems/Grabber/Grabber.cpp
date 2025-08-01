// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Grabber.h"

Grabber::Grabber() {
}

void Grabber::setMotorVoltage(units::volt_t voltage) {
    grabberMotor.SetControl(grabberVoltage.WithOutput(voltage).WithEnableFOC(true));
}

double Grabber::getVoltage() {
    return grabberMotor.GetMotorVoltage().GetValueAsDouble();
}

bool Grabber::isCoralIn() {

    return canRange.GetIsDetected().GetValue();
}

bool Grabber::isAlgaeIn() {

    return canRange.GetIsDetected().GetValue();

    //return units::math::abs(intakeMotor.GetSupplyCurrent().GetValue()) > 31.0_A;

}

frc2::CommandPtr Grabber::moveGrabber(units::volt_t voltage) {
    return this->RunOnce([this, voltage] {
        this->setMotorVoltage(voltage);
    });
}

void Grabber::Periodic() {
    frc::SmartDashboard::PutBoolean("Grabber/ACTIVATED?", getVoltage() > 0.0);
    frc::SmartDashboard::PutBoolean("Sensor Activated???", isCoralIn());
}
