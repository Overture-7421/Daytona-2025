#include "Subsystems/Arm/Arm.h"

Arm::Arm() {
    frc::SmartDashboard::PutNumber("Arm/TargetArmAngle", 0.0);
    frc::SmartDashboard::PutBoolean("Arm/AtPosition", false);

    armMotor.setSensorToMechanism(ArmConstants::ArmRotorToSensor);
    //armMotor.setFusedCANCoder(ArmConstants::ArmCANCoderId);
    armMotor.SetPosition(armCANCoder.GetAbsolutePosition().GetValue());
    armMotor.configureMotionMagic(ArmConstants::ArmCruiseVelocity, ArmConstants::ArmCruiseAcceleration, 0_tr_per_s_cu);

}

frc2::CommandPtr Arm::setArmZero() {
    return frc2::cmd::RunOnce([this]() {
        armMotor.SetPosition(armCANCoder.GetAbsolutePosition().GetValue());
    });
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

frc2::CommandPtr Arm::setState(Positions state, Heading heading) {
    return frc2::FunctionalCommand([this, state, heading]() {
        if (heading == Heading::Front) {
            setToAngle(ArmConstants::ArmFront.at(state));
        } else if (heading == Heading::Back) {
            setToAngle(ArmConstants::ArmBack.at(state));
        }
    }, []() {
    }, [](bool interupted) {
    }, [this, state, heading]() {
        if (heading == Heading::Front) {
            frc::SmartDashboard::PutBoolean("Arm/AtPosition", isArmAtPosition(ArmConstants::ArmFront.at(state)));
            return isArmAtPosition(ArmConstants::ArmFront.at(state));
        } else if (heading == Heading::Back) {
            frc::SmartDashboard::PutBoolean("Arm/AtPosition", isArmAtPosition(ArmConstants::ArmBack.at(state)));
            return isArmAtPosition(ArmConstants::ArmBack.at(state));
        }

        return true;
    },
    {this}).ToPtr();
}

frc2::CommandPtr Arm::setCharacterization(units::degree_t angle) {
    return frc2::FunctionalCommand([this, angle]() {
        setToAngle(angle);
    }, []() {
    }, [](bool interupted) {
    }, [this, angle]() {
        return isArmAtPosition(angle);
    },
    {this}).ToPtr();
}

frc2::CommandPtr Arm::setState(Positions state) {
    return frc2::FunctionalCommand([this, state]() {
        setToAngle(ArmConstants::ArmFront.at(state));
    }, []() {
    }, [](bool interupted) {
    }, [this, state]() {
        frc::SmartDashboard::PutBoolean("Arm/AtPosition", isArmAtPosition(ArmConstants::ArmFront.at(state)));
        return isArmAtPosition(ArmConstants::ArmFront.at(state));
    },
    {this}).ToPtr();
}

void Arm::Periodic() {

    frc::SmartDashboard::PutNumber("Arm/CurrentArmAngle", getCurrentAngle().value());

}
