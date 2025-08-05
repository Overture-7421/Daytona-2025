// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Grabber.h"

Grabber::Grabber() {
}

//Gives the desired voltage to the grabber
void Grabber::setMotorVoltage(units::volt_t voltage) {
    grabberMotor.SetControl(grabberVoltage.WithOutput(voltage).WithEnableFOC(true));
}

//Checks how much Voltage is 
double Grabber::getVoltage() {
    return grabberMotor.GetMotorVoltage().GetValueAsDouble();
}

bool Grabber::isCoralIn() {

    return grabberMotor.GetSupplyCurrent().GetValueAsDouble() > GrabberConstants::CoralDetectionCurrent;

}

bool Grabber::isAlgaeIn() {

    return grabberMotor.GetSupplyCurrent().GetValueAsDouble() > GrabberConstants::AlgaeDetectionCurrent;

}

//Command that only makes the Grabber move by giving it the desired voltage
frc2::CommandPtr Grabber::moveGrabber(units::volt_t voltage) {
    return this->RunOnce([this, voltage] {
        this->setMotorVoltage(voltage);
    });
}

void Grabber::Periodic() {

    frc::SmartDashboard::PutBoolean("Grabber/ACTIVATED?", getVoltage() > 0.0);
    frc::SmartDashboard::PutBoolean("Sensor Activated???", isCoralIn());

}
