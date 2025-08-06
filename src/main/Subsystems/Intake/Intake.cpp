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

frc2::CommandPtr Intake::setIntakeCommand(units::degree_t intakeAngle, units::volt_t rollersVolts,
        units::volt_t centeringVolts) {
    return frc2::FunctionalCommand([this, intakeAngle]() {
        setIntakeToAngle(intakeAngle);
    }, [this, rollersVolts, centeringVolts]() {
        setRollersVoltage(rollersVolts);
        setCenteringVoltage(centeringVolts);
    },[](bool interrupted) {
    },[this, intakeAngle]() {
        return isIntakeAtPosition(intakeAngle);
    },
    {this}).ToPtr();
}

void Intake::Periodic() {
    frc::SmartDashboard::PutBoolean("Sensor Activated???", isCoralIn());

}
