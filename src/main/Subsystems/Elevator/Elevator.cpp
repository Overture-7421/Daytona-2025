// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Elevator.h"

Elevator::Elevator() {

    rightElevatorMotor.setFollow(leftElevatorMotor.GetDeviceID(), true);  //Makes the right elevator to follow left.

    rightElevatorMotor.SetPosition(0_tr); //Is the beginning value assigned to both motors
    leftElevatorMotor.SetPosition(0_tr);

    leftElevatorMotor.setSensorToMechanism(ElevatorConstants::LowerSensorToMechanism); //Assignation of "SensorToMechanism" values
    rightElevatorMotor.setSensorToMechanism(ElevatorConstants::LowerSensorToMechanism);
 
    //Configuration of Speed, Acceleration and Jerk given when the elevator starts
    leftElevatorMotor.configureMotionMagic(ElevatorConstants::ElevatorCruiseVelocity,
            ElevatorConstants::ElevatorUpperCruiseAcceleration, 0.0_tr_per_s_cu);
}

//Function that permits us to provide a target position to the elevator 
void Elevator::setTarget(units::meter_t position) {
    frc::SmartDashboard::PutNumber("Elevator/TargetPosition", position.value());

    units::turn_t transformedPosition {position.value() / (ElevatorConstants::Diameter.value() * M_PI)};

    leftElevatorMotor.SetControl(
            elevatorVoltage.WithPosition(transformedPosition).WithEnableFOC(true).WithFeedForward(
                    ElevatorConstants::feedForward));
}

//Functions that alterate the inner configurationsof the motors depending of wether it is going up or down.
void Elevator::setElevatorUpperSpeed() {
    leftElevatorMotor.configureMotionMagic(ElevatorConstants::ElevatorCruiseVelocity,
            ElevatorConstants::ElevatorUpperCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Elevator::setElevatorLowerSpeed() {
    leftElevatorMotor.configureMotionMagic(ElevatorConstants::ElevatorCruiseVelocity,
            ElevatorConstants::ElevatorLowerCruiseAcceleration, 0.0_tr_per_s_cu);
}

//Retrieves the current position of the elevator.
units::meter_t Elevator::getPosition() {
    units::meter_t currentPosition = units::meter_t(
            leftElevatorMotor.GetPosition().GetValueAsDouble() * (ElevatorConstants::Diameter.value() * M_PI));
    return currentPosition;
}

//Function that checks if the current position of the elevator is within the range of error or not.
bool Elevator::isElevatorAtPosition(units::meter_t elevatorPosition) {
    units::meter_t elevatorError = elevatorPosition
            - units::meter_t(
                    leftElevatorMotor.GetPosition().GetValueAsDouble() * (ElevatorConstants::Diameter.value() * M_PI));
    return (units::math::abs(elevatorError) < ElevatorConstants::RangeError);
}

//Command pointer which allows all the true actions and movement to occur
frc2::CommandPtr Elevator::setElevatorCommand(units::meter_t elevatorPosition) {
    return frc2::FunctionalCommand([this, elevatorPosition]() {
        setTarget(elevatorPosition);
    },
    []() {
    },
    [](bool interrupted) {
    },
    [this, elevatorPosition]() {
        return isElevatorAtPosition(elevatorPosition);
    },
    {this}).ToPtr();
}

// This method will be called once per scheduler run
void Elevator::Periodic() {

    frc::SmartDashboard::PutNumber("ElevatorCurrent/CurrentElevatorMotor", getPosition().value());

}
