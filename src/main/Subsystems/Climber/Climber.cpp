// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Climber.h"
#include <iostream>

Climber::Climber(){

    armMotor.setSensorToMechanism(Constants::ArmSensorToMechanism); 
    hookMotor.setSensorToMechanism(Constants::HookSensorToMechanism);

    armMotor.configureMotionMagic(Constants::ArmCruiseVelocity, Constants::ArmCruiseAcceleration, 0.0_tr_per_s_cu);
    hookMotor.configureMotionMagic(Constants::HookCruiseVelocity, Constants::HookCruiseAcceleration, 0.0_tr_per_s_cu);
}

//Function that requires an angle and does calculations with other values in order to get or to order the mechanism to go to a position.
void Climber::setToAngle(units::degree_t armAngle, units::degree_t hookAngle) {

    auto armTurns = units::turn_t(armMotor.GetClosedLoopReference().GetValueAsDouble());
    auto armTurnsPerSecond = units::turns_per_second_t(armMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

    auto hookTurns = units::turn_t(hookMotor.GetClosedLoopReference().GetValueAsDouble());
    auto hookTurnsPerSecond = units::turns_per_second_t(hookMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

    armMotor.SetControl(armVoltage.WithPosition(armAngle).WithFeedForward(armFeedForward.Calculate(armTurns, armTurnsPerSecond)).WithEnableFOC(true));
    hookMotor.SetControl(hookVoltage.WithPosition(hookAngle).WithFeedForward(hookFeedForward.Calculate(hookTurns, hookTurnsPerSecond)).WithEnableFOC(true));
};

bool Climber::isClimberAtPosition(units::degree_t armAngle, units::degree_t hookAngle){
    units::degree_t armError = armAngle - armMotor.GetPosition().GetValue();
    units::degree_t hookError = hookAngle - hookMotor.GetPosition().GetValue();

    if(units::math::abs(armError) < 1.0_deg && units::math::abs(hookError) < 1.0_deg ){
        return true;
    } else {
        return false;
    }
}


frc2::CommandPtr Climber::setClimberCommand(units::degree_t armAngle, units::degree_t hookAngle){
    return frc2::FunctionalCommand(
		[&]() {setToAngle(armAngle, hookAngle);},
		[&]() {},
		[&](bool interupted) {},
		[&]() {return isClimberAtPosition(armAngle, hookAngle);},
		{ this }
	).ToPtr();
}



frc2::CommandPtr Climber::SysIdQuasistatic(frc2::sysid::Direction direction){
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Climber::SysIdDynamic(frc2::sysid::Direction direction){
  return m_sysIdRoutine.Dynamic(direction);
}

void Climber::Periodic() {

    double armCurrentAngle = armMotor.GetPosition().GetValueAsDouble();
    double hookCurrentAngle = hookMotor.GetPosition().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("Current Arm Angle", armCurrentAngle);
    frc::SmartDashboard::PutNumber("Current hook Angle", hookCurrentAngle);
}