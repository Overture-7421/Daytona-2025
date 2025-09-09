// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Intake/Intake.h"

Intake::Intake() {
    intakeMotor.setRotorToSensorRatio(IntakeConstants::IntakeRotorToSensor);
    intakeMotor.setFusedCANCoder(IntakeConstants::IntakeCANCoderId);

    intakeMotor.configureMotionMagic(IntakeConstants::IntakeCruiseVelocity, IntakeConstants::IntakeCruiseAcceleration,
            0.0_tr_per_s_cu);

    frc::SmartDashboard::PutNumber("Intake/TargetIntakeAngle", 0.0);
    frc::SmartDashboard::PutBoolean("Intake/IsFinished", false);

}

void Intake::setIntakeToAngle(units::degree_t intakeAngle) {
    frc::SmartDashboard::PutNumber("Intake/TargetIntakeAngle", intakeAngle.value());
    intakeMotor.SetControl(intakeVoltage.WithPosition(intakeAngle).WithEnableFOC(true));
}

bool Intake::isIntakeAtPosition(units::degree_t intakeAngle) {
    units::degree_t intakeError = intakeAngle - intakeMotor.GetPosition().GetValue();
    frc::SmartDashboard::PutNumber("Intake/IntakeError", intakeError.value());
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

units::degree_t Intake::getIntakePosition() {
    return intakeMotor.GetPosition().GetValue();
}

frc2::CommandPtr Intake::setState(Positions state) {
    return frc2::FunctionalCommand([this, state]() {
        setIntakeToAngle(IntakeConstants::IntakePositions.at(state).intake);
    }, [this, state]() {

        if (!(state == Positions::CoralHold)) {
            setRollersVoltage(IntakeConstants::IntakePositions.at(state).rollers);
            setCenteringVoltage(IntakeConstants::IntakePositions.at(state).centering);
        }
    }, [](bool interrupted) {
    }
            , [this, state]() {
                frc::SmartDashboard::PutBoolean("Intake/AtPosition",
                        isIntakeAtPosition(IntakeConstants::IntakePositions.at(state).intake));
                return isIntakeAtPosition(IntakeConstants::IntakePositions.at(state).intake);
            },
            {this}).ToPtr().BeforeStarting([this, state]() {
        return frc::SmartDashboard::PutBoolean("Intake/IsFinished", false);
    }).AndThen([this, state]() {
        if (state == Positions::CoralHold) {
            setRollersVoltage(IntakeConstants::IntakePositions.at(state).rollers);
            setCenteringVoltage(IntakeConstants::IntakePositions.at(state).centering);
        }
        frc::SmartDashboard::PutBoolean("Intake/IsFinished", true);
    });
}

frc2::CommandPtr Intake::setCharacterization(units::volt_t rollers, units::volt_t centering, units::degree_t intake) {
    return frc2::FunctionalCommand([this, intake]() {
        setIntakeToAngle(intake);
    }, [this, rollers, centering]() {

        setRollersVoltage(rollers);
        setCenteringVoltage(centering);

    }, [](bool interrupted) {
    }, [this, intake]() {
        frc::SmartDashboard::PutBoolean("Intake/AtPosition", isIntakeAtPosition(intake));
        return isIntakeAtPosition(intake);
    },
    {this}).ToPtr();
}

void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake/CurrentIntakeAngle", intakeMotor.GetPosition().GetValueAsDouble() * 360);
    frc::SmartDashboard::PutBoolean("Intake/CoralIn", isCoralIn());

}
