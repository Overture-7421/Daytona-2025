// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() = default;

void Intake::setVoltage(units::volt_t voltage){
    intakeMotor.SetVoltage(voltage);
}

double Intake::getVoltage(){
    return intakeMotor.GetMotorVoltage().GetValueAsDouble();
}
//...

frc2::CommandPtr Intake::moveIntake(units::volt_t voltage){return this->RunOnce([this, voltage] {this->setVoltage(voltage);});};


// This method will be called once per scheduler run
void Intake::Periodic() {
  frc::SmartDashboard::PutBoolean("Intake/ACTIVATED?", getVoltage() > 0.0);
}
