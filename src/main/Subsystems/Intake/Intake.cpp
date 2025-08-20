// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Intake/Intake.h"

Intake::Intake() {
    intakeMotor.setRotorToSensorRatio(IntakeConstants::IntakeRotorToSensor);
    intakeMotor.setFusedCANCoder(IntakeConstants::IntakeCANCoderId);

    intakeMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration,
            0.0_tr_per_s_cu);
}

void Intake::setIntakeToAngle(units::degree_t intakeAngle) {
    intakeMotor.SetControl(intakeVoltage.WithPosition(intakeAngle).WithEnableFOC(true));
}

bool Intake::isIntakeAtPosition(units::degree_t intakeAngle) {
    units::degree_t intakeError = intakeMotor.GetPosition().GetValue();
    return (units::math::abs(intakeError) < IntakeConstants::IntakeRangeError);
}

void Intake::setRollersVoltage(units::volt_t voltage) {
    rollersMotor.SetControl(rollersVoltage.WithOutput(voltage).WithEnableFOC(true));
}

void Intake::setCenteringVoltage(units::volt_t voltage) {
    centeringMotor.SetControl(centeringVoltage.WithOutput(voltage).WithEnableFOC(true));
}

bool Intake::isCoralIn() {
    return canRange.GetIsDetected().GetValue();
}

frc2::CommandPtr Intake::setState(Positions state) {
    return frc2::FunctionalCommand([this, state]() {
        setIntakeToAngle(IntakeConstants::IntakePositions.at(state).intake);
    }, [this, state]() {
        setRollersVoltage(IntakeConstants::IntakePositions.at(state).rollers);
        setCenteringVoltage(IntakeConstants::IntakePositions.at(state).centering);
    },[](bool interrupted) {
    },[this, state]() {
        return isIntakeAtPosition(IntakeConstants::IntakePositions.at(state).intake);
    },
    {this}).ToPtr();
}

void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Sensor Activated???", isCoralIn());

}
