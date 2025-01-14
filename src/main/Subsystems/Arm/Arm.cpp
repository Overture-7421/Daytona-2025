// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/Arm/Arm.h"
#include <iostream>

Arm::Arm(){
    armRightMotor.setFollow(Constants::ArmLeftMotorId, true);

    armLeftMotor.setRotorToSensorRatio(Constants::ArmRotorToSensor);
    wristMotor.setRotorToSensorRatio(Constants::WristRotorToSensor);  

    armLeftMotor.setSensorToMechanism(Constants::ArmSensorToMechanism); 
    wristMotor.setSensorToMechanism(Constants::WristSensorToMechanism);   

    armLeftMotor.setFusedCANCoder(Constants::ArmCANCoderId);
    wristMotor.setFusedCANCoder(Constants::WristCANCoderId);

    armLeftMotor.configureMotionMagic(Constants::ArmCruiseVelocity, Constants::ArmCruiseAcceleration, 0.0_tr_per_s_cu);
    wristMotor.configureMotionMagic(Constants::WristCruiseVelocity, Constants::WristCruiseAcceleration, 0.0_tr_per_s_cu);
}


void Arm::setToAngle(units::degree_t armAngle, units::degree_t wristAngle) {
    auto armTurns = units::turn_t(armLeftMotor.GetClosedLoopReference().GetValueAsDouble());
    auto armTurnsPerSecond = units::turns_per_second_t(armLeftMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

    auto wristTurns = units::turn_t(wristMotor.GetClosedLoopReference().GetValueAsDouble());
    auto wristTurnsPerSecond = units::turns_per_second_t(wristMotor.GetClosedLoopReferenceSlope().GetValueAsDouble());

    armLeftMotor.SetControl(armVoltage.WithPosition(armAngle).WithFeedForward(armFeedForward.Calculate(armTurns, armTurnsPerSecond)).WithEnableFOC(true));
    wristMotor.SetControl(wristVoltage.WithPosition(wristAngle).WithFeedForward(wristFeedForward.Calculate(wristTurns, wristTurnsPerSecond)).WithEnableFOC(true));
};


bool Arm::isArmAtPosition(units::degree_t armAngle, units::degree_t wristAngle){
    units::degree_t armError = armAngle - armLeftMotor.GetPosition().GetValue();
    units::degree_t wristError = wristAngle - wristMotor.GetPosition().GetValue();

    if(units::math::abs(armError) < 1.0_deg && units::math::abs(wristError) < 1.0_deg ){
        return true;
    } else {
        return false;
    }
}


frc2::CommandPtr Arm::setArmCommand(units::degree_t armAngle, units::degree_t wristAngle){
    return frc2::FunctionalCommand(
		[&]() {setToAngle(armAngle, wristAngle);},
		[&]() {},
		[&](bool interupted) {},
		[&]() {return isArmAtPosition(armAngle, wristAngle);},
		{ this }
	).ToPtr();
}



frc2::CommandPtr Arm::SysIdQuasistatic(frc2::sysid::Direction direction){
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Arm::SysIdDynamic(frc2::sysid::Direction direction){
  return m_sysIdRoutine.Dynamic(direction);
}

void Arm::Periodic() {
    double armCurrentAngle = armCANCoder.GetPosition().GetValueAsDouble();
    double wristCurrentAngle = wristCANCoder.GetPosition().GetValueAsDouble();
    frc::SmartDashboard::PutNumber("Current Arm Lower Angle", armCurrentAngle);
    frc::SmartDashboard::PutNumber("Current Arm Upper Angle", wristCurrentAngle);
}
