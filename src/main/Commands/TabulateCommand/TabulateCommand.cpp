// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TabulateCommand.h"

TabulateCommand::TabulateCommand(Elevator *elevator, Arm *arm, Intake *intake) {
    this->elevator = elevator;
    this->arm = arm;
    this->intake = intake;

    AddRequirements( {elevator, arm, intake});
}

void TabulateCommand::Initialize() {
    frc::SmartDashboard::PutNumber("Tabulate/elevatorPos", elevator->getPosition().value());
    frc::SmartDashboard::PutNumber("Tabulate/armAngle", arm->getArmAngle().value());
    frc::SmartDashboard::PutNumber("Tabulate/wristAngle", arm->getWristAngle().value());
    frc::SmartDashboard::PutNumber("Tabulate/jawAngle", intake->getJawAngle().value());
    frc::SmartDashboard::PutNumber("Tabulate/intakeVolts", intake->getVoltage());

}

void TabulateCommand::Execute() {

    frc::SmartDashboard::PutNumber("Tabulate/elevatorPosCurrent", elevator->getPosition().value());
    frc::SmartDashboard::PutNumber("Tabulate/armAngleCurrent", arm->getArmAngle().value());
    frc::SmartDashboard::PutNumber("Tabulate/wristAngleCurrent", arm->getWristAngle().value());
    frc::SmartDashboard::PutNumber("Tabulate/jawAngleCurrent", intake->getJawAngle().value());
    frc::SmartDashboard::PutNumber("Tabulate/intakeVoltsCurrent", intake->getVoltage());

    units::meter_t elevatorPos {frc::SmartDashboard::GetNumber("Tabulate/elevatorPos", elevator->getPosition().value())};
    units::degree_t armAngle {frc::SmartDashboard::GetNumber("Tabulate/armAngle", arm->getArmAngle().value())};
    units::degree_t wristAngle {frc::SmartDashboard::GetNumber("Tabulate/wristAngle", arm->getWristAngle().value())};
    units::degree_t jawAngle {frc::SmartDashboard::GetNumber("Tabulate/jawAngle", intake->getJawAngle().value())};
    units::volt_t intakeVolts {frc::SmartDashboard::GetNumber("Tabulate/intakeVolts", intake->getVoltage())};

    elevator->setPosition(elevatorPos);
    arm->setToAngle(armAngle, wristAngle);
    intake->setToAngle(jawAngle);
    intake->setMotorVoltage(intakeVolts);

}

void TabulateCommand::End(bool interrupted) {
}

bool TabulateCommand::IsFinished() {
    return false;
}
