// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Elevator.h"

Elevator::Elevator(){
    rightElevatorMotor.setFollow(9, true);

    leftElevatorMotor.setSensorToMechanism(ElevatorConstants::LowerSensorToMechanism);

}

void Elevator::setPosition(units::meter_t position){

    units::turn_t positionInRotations{position.value() / (ElevatorConstants::diameter.value() * 3.14159265358979323846)};
    leftElevatorMotor.SetControl(elevatorVoltage.WithPosition(positionInRotations).WithEnableFOC(true));
    
};

bool Elevator::isElevatorAtPosition(units::meter_t elevatorPosition){
    units::meter_t elevatorError = elevatorPosition - leftElevatorMotor.GetPosition().GetValueAsDouble();
     if(units::math::abs(elevatorError) < 1.0_deg) {
        return true;
     } else {
        return false;
     }
}

frc2::CommandPtr Elevator::setElevatorCommand(units::meter_t elevatorPosition){
    return frc2::FunctionalCommand(
        [&]() {setPosition(elevatorPosition);},
        [&]() {},
        [&](bool interrupted) {},
        [&]() {return isElevatorAtPosition(elevatorPosition);},
        { this }
    ).ToPtr();
}

// This method will be called once per scheduler run
void Elevator::Periodic() {

    //no sé si está bien jiji
    double currentPosition = leftElevatorMotor.GetPosition().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("Current Elevator Position", currentPosition);

}
