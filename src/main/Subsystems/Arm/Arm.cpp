#include "Subsystems/Arm/Arm.h"

Arm::Arm() {

    armMotor.setRotorToSensorRatio(ArmConstants::ArmRotorToSensor);
    armMotor.setFusedCANCoder(ArmConstants::ArmCANCoderId);
    armMotor.configureMotionMagic(ArmConstants::ArmCruiseVelocity, ArmConstants::ArmCruiseAcceleration, 0_tr_per_s_cu);

}

void Arm::setToAngle(units::degree_t armAngle) {
    frc::SmartDashboard::PutNumber("Arm/TargetArmAngle", armAngle.value());
    armMotor.SetControl(armVoltage.WithPosition(armAngle).WithEnableFOC(true));
}

units::degree_t Arm::getCurrentAngle() {
    return armMotor.GetPosition().GetValue();
}

bool Arm::isArmAtPosition(units::degree_t armAngle) {
    units::degree_t armError = armAngle - armMotor.GetPosition().GetValue();
    return (units::math::abs(armError) < ArmConstants::ArmRangeError);
}

frc2::CommandPtr Arm::setArmCommand(units::degree_t armAngle) {
    return frc2::FunctionalCommand([this, armAngle]() {
        setToAngle(armAngle);
    }, []() {
    }, [](bool interupted) {
    }, [this, armAngle]() {
        return isArmAtPosition(armAngle);
    },
    {this}).ToPtr();
}

void Arm::Periodic() {

}
