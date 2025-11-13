// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Climber/Climber.h"

Climber::Climber() {

    climberMotor.setRemoteCANCoder(ClimberConstants::climberCANCoderId);
    climberMotor.configureMotionMagic(ClimberConstants::ClimberVelocity, ClimberConstants::ClimberAcceleration,
            0.0_tr_per_s_cu);
    frc::SmartDashboard::PutBoolean("Climber/AtPosition", false);
}

void Climber::setToAngle(units::degree_t climberAngle) {
    frc::SmartDashboard::PutNumber("Climber/Target", climberAngle.value());

    climberMotor.SetControl(armVoltage.WithPosition(climberAngle).WithEnableFOC(true));

}

bool Climber::isClimberAtPosition(units::degree_t climberAngle) {
    units::degree_t armError = climberAngle - climberCANCoder.GetAbsolutePosition().GetValue();
    frc::SmartDashboard::PutNumber("Climber/Error", armError.value());
    return (units::math::abs(armError) < ClimberConstants::ClimberRangeError);

}

frc2::CommandPtr Climber::setClimberCommand(units::degree_t climberAngle) {
    return frc2::FunctionalCommand([this, climberAngle]() {
        if (!isDisabled) {
            setToAngle(climberAngle);
        }
    }, [this, climberAngle]() {
        if (!isDisabled) {
            setToAngle(climberAngle + offset);
        }
    }, [this](bool interupted) {
        offset = 0_deg;
    }, [this, climberAngle]() {
        frc::SmartDashboard::PutBoolean("Climber/AtPosition", isClimberAtPosition(climberAngle));
        if (isDisabled) {
            return true;
        }
        return isClimberAtPosition(climberAngle);
        // return true;
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::setClimberClimbedCommand(units::degree_t climberAngle) {
    return frc2::FunctionalCommand([this, climberAngle]() {
        if (!isDisabled) {
            setToAngle(climberAngle);
        }
    }, [this, climberAngle]() {
        if (!isDisabled) {
            setToAngle(climberAngle + offset);
        }
    }, [this](bool interupted) {
        offset = 0_deg;
    }, [this]() {
        if (isDisabled) {
            return true;
        }
        return false;
        // return true;
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::disableClimberCommand() {
    return frc2::FunctionalCommand([this]() {
        climberMotor.Disable();
        isDisabled = true;
    }, []() {

    }, [](bool interupted) {

    }, []() {
        return true;
    },
    {this}).ToPtr();
}

void Climber::setOffset() {
    offset += 2_deg;
}

void Climber::Periodic() {
    //frc::SmartDashboard::PutNumber("Climber/CurrentThroughbore",climberCANCoder.GetAbsolutePosition().GetValue().value() * 360);
}

