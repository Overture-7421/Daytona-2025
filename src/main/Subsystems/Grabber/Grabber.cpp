// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Grabber.h"

Grabber::Grabber() {
}

//Gives the desired voltage to the grabber
void Grabber::setMotorVoltage(units::volt_t voltage) {
    grabberMotor.SetControl(grabberVoltage.WithOutput(voltage).WithEnableFOC(true));
    frc::SmartDashboard::PutBoolean("Grabber/IsFinished", false);
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

frc2::CommandPtr Grabber::setState(Positions state) {
    return frc2::cmd::RunOnce([this, state]() {
        setMotorVoltage(GrabberConstants::GrabberVoltage.at(state));
    }).BeforeStarting([this]() {
        return frc::SmartDashboard::PutBoolean("Grabber/IsFinished", false);
    }).AndThen([this]() {
        return frc::SmartDashboard::PutBoolean("Grabber/IsFinished", true);
    });
}

frc2::CommandPtr Grabber::setCharacterization(units::volt_t voltage) {
    return frc2::cmd::RunOnce([this, voltage]() {
        setMotorVoltage(voltage);
    });
}

void Grabber::Periodic() {

    frc::SmartDashboard::PutBoolean("Grabber/ACTIVATED?", getVoltage() > 0.0);
    frc::SmartDashboard::PutBoolean("Grabber/CoralIn", isCoralIn());
    frc::SmartDashboard::PutBoolean("Grabber/AlgaeIn", isAlgaeIn());

}
