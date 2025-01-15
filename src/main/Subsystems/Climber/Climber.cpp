// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"
#include <iostream>

Climber::Climber() {

    armLeftMotor.setFollow(armRightMotor.GetDeviceID(), true);

    armRightMotor.setSensorToMechanism(Constants::ArmSensorToMechanism);
    armRightMotor.configureMotionMagic(Constants::ArmCruiseVelocity,
            Constants::ArmCruiseAcceleration, 0.0_tr_per_s_cu);

}

//Function that requires an angle and does calculations with other values in order to get or to order the mechanism to go to a position.
void Climber::setToAngle(units::degree_t armAngle) {

    armRightMotor.SetControl(
            armVoltage.WithPosition(armAngle).WithEnableFOC(true));

}

bool Climber::isClimberAtPosition(units::degree_t armAngle) {
    units::degree_t armError = armAngle
            - armRightMotor.GetPosition().GetValue();

    return units::math::abs(armError) < 1.0_deg;

}

frc2::CommandPtr Climber::setClimberCommand(units::degree_t armAngle) {
    return frc2::FunctionalCommand([&]() {
        setToAngle(armAngle);
    },
    [&]() {
    },
    [&](bool interupted) {
    },
    [&]() {
        return isClimberAtPosition(armAngle);
    },
    { this }).ToPtr();
}

frc2::CommandPtr Climber::SysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Climber::SysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Dynamic(direction);
}

void Climber::Periodic() {

    double armCurrentAngle = armRightMotor.GetPosition().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("Climber/Current Arm Angle",
            armCurrentAngle);
}
