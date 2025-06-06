// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Elevator.h"

Elevator::Elevator() {
    rightElevatorMotor.setFollow(leftElevatorMotor.GetDeviceID(), true);

    rightElevatorMotor.SetPosition(0_tr);
    leftElevatorMotor.SetPosition(0_tr);

    leftElevatorMotor.setSensorToMechanism(ElevatorConstants::LowerSensorToMechanism);
    rightElevatorMotor.setSensorToMechanism(ElevatorConstants::LowerSensorToMechanism);

    leftElevatorMotor.configureMotionMagic(ElevatorConstants::ElevatorCruiseVelocity,
            ElevatorConstants::ElevatorCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Elevator::setPosition(units::meter_t position) {
    frc::SmartDashboard::PutNumber("Elevator/TargetPosition", position.value());

    units::turn_t positionInRotations {position.value() / (ElevatorConstants::Diameter.value() * M_PI)};
    leftElevatorMotor.SetControl(
            elevatorVoltage.WithPosition(positionInRotations).WithEnableFOC(true).WithFeedForward(
                    ElevatorConstants::feedForward));

}

void Elevator::setElevatorNormalSpeed() {
    leftElevatorMotor.configureMotionMagic(ElevatorConstants::ElevatorCruiseVelocity,
            ElevatorConstants::ElevatorCruiseAcceleration, 0.0_tr_per_s_cu);
}

void Elevator::setElevatorLowerSpeed() {
    leftElevatorMotor.configureMotionMagic(ElevatorConstants::ElevatorLowerCruiseVelocity,
            ElevatorConstants::ElevatorLowerCruiseAcceleration, 0.0_tr_per_s_cu);
}

units::meter_t Elevator::getPosition() {
    units::meter_t currentPosition = units::meter_t(
            leftElevatorMotor.GetPosition().GetValueAsDouble() * (ElevatorConstants::Diameter.value() * M_PI));
    return currentPosition;
}

bool Elevator::isElevatorAtPosition(units::meter_t elevatorPosition) {
    units::meter_t elevatorError = elevatorPosition
            - units::meter_t(
                    leftElevatorMotor.GetPosition().GetValueAsDouble() * (ElevatorConstants::Diameter.value() * M_PI));
    return (units::math::abs(elevatorError) < ElevatorConstants::RangeError);

}

frc2::CommandPtr Elevator::setElevatorCommand(units::meter_t elevatorPosition) {
    return frc2::FunctionalCommand([this, elevatorPosition]() {
        setPosition(elevatorPosition);
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
