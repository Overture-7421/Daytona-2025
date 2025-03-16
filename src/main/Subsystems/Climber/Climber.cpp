// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"
#include <iostream>

Climber::Climber() {

    armRightMotor.SetPosition(0_tr);
    armRightMotor.setSensorToMechanism(ClimberConstants::ArmSensorToMechanism);
    armRightMotor.configureMotionMagic(ClimberConstants::ArmCruiseVelocity, ClimberConstants::ArmCruiseAcceleration,
            0.0_tr_per_s_cu);

}

//Function that requires an angle and does calculations with other values in order to get or to order the mechanism to go to a position.
void Climber::setToAngle(units::degree_t armAngle) {
    frc::SmartDashboard::PutNumber("Climber/TargetArmAngle", armAngle.value());

    armRightMotor.SetControl(armVoltage.WithPosition(armAngle).WithEnableFOC(true));

}

bool Climber::isClimberAtPosition(units::degree_t armAngle) {
    units::degree_t armError = armAngle - armRightMotor.GetPosition().GetValue();

    return (units::math::abs(armError) < 2.0_deg);

}

frc2::CommandPtr Climber::setClimberCommand(units::degree_t armAngle) {
    return frc2::FunctionalCommand([this, armAngle]() {
        setToAngle(armAngle);
    }, []() {
    }, [](bool interupted) {
    }, [this, armAngle]() {
        return isClimberAtPosition(armAngle);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Climber::SysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Climber::SysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Dynamic(direction);
}

void Climber::Periodic() {

    units::degree_t climberCurrentAngleMotor = armRightMotor.GetPosition().GetValue();
    frc::SmartDashboard::PutNumber("ClimberCurrent/CurrentClimberAngleMotor", climberCurrentAngleMotor.value());

}
