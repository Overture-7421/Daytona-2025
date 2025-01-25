// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmMotion.h"

ArmMotion::ArmMotion(Elevator *elevator, Arm *arm, units::degree_t armAngle, units::degree_t wristAngle,
        units::meter_t elevatorPosition) {
    this->elevator = elevator;
    this->arm = arm;
    this->armAngle = armAngle;
    this->wristAngle = wristAngle;
    this->elevatorPosition = elevatorPosition;
    AddRequirements( {arm});
}

// Called when the command is initially scheduled.
void ArmMotion::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ArmMotion::Execute() {

    if (elevator->getPosition() < ElevatorConstants::HighMotionAllowed) {
        arm->blockedWrist(armAngle, wristAngle);
    } else {
        arm->setArmCommand(armAngle, wristAngle);
    }
}

// Called once the command ends or is interrupted.
void ArmMotion::End(bool interrupted) {
}

// Returns true when the command should end.
bool ArmMotion::IsFinished() {
    return arm->isArmAtPosition(armAngle, wristAngle) && elevator->isElevatorAtPosition(elevatorPosition);
}
