// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ArmHighMotion.h"

ArmHighMotion::ArmHighMotion(Elevator* elevator, Arm* arm, units::degree_t armAngle, units::degree_t wristAngle, units::meter_t elevatorPosition) {
  this-> elevator = elevator;
  this-> arm = arm;
  this-> armAngle = armAngle;
  this-> wristAngle = wristAngle;
  this-> elevatorPosition = elevatorPosition;
  AddRequirements({elevator, arm});
}

// Called when the command is initially scheduled.
void ArmHighMotion::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ArmHighMotion::Execute() {

  if (elevator->getPosition() > 0.43){
    arm-> setArmCommand( armAngle,  wristAngle);
  } else{
    arm-> blockedWrist( armAngle, wristAngle);
  }
}

// Called once the command ends or is interrupted.
void ArmHighMotion::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmHighMotion::IsFinished() {
  return arm->isArmAtPosition(armAngle, wristAngle) && elevator->isElevatorAtPosition(elevatorPosition);
}
