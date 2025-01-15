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
    MotionMagicVoltage armVoltage{0_tr};
    MotionMagicVoltage wristVoltage{0_tr};

    armLeftMotor.SetControl(armVoltage.WithPosition(armAngle).WithEnableFOC(true));
    wristMotor.SetControl(wristVoltage.WithPosition(wristAngle).WithEnableFOC(true));
};


bool Arm::isArmAtPosition(units::degree_t armAngle, units::degree_t wristAngle){
    units::degree_t armError = armAngle - armLeftMotor.GetPosition().GetValue();
    units::degree_t wristError = wristAngle - wristMotor.GetPosition().GetValue();

    return units::math::abs(armError) < Constants::ArmAngleRange && units::math::abs(wristError) < Constants::WristAngleRange;
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
    frc::SmartDashboard::PutNumber("Arm/Current Arm Angle", armCurrentAngle);
    frc::SmartDashboard::PutNumber("Arm/Current Wrist Angle", wristCurrentAngle);
}
