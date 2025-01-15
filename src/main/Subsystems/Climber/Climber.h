// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/FunctionalCommand.h>
#include "Subsystems/Climber/Constants.h"

class Climber: public frc2::SubsystemBase
{
public:
    Climber();

    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

    void setToAngle(units::degree_t armAngle);
    frc2::CommandPtr setClimberCommand(units::degree_t armAngle);

    bool isClimberAtPosition(units::degree_t armAngle);
    void getCurrentAngle(double armAngle);

    void Periodic() override;

private:
    OverTalonFX armRightMotor
    { Constants::RightConfig(), "rio" };
    OverTalonFX armLeftMotor
    { Constants::LeftConfig(), "rio" };

    frc2::sysid::SysIdRoutine m_sysIdRoutine
    {   frc2::sysid::Config
        {   1_V / 1_s, 3_V, 30_s, nullptr},
        frc2::sysid::Mechanism
        {
            [this](units::volt_t driveVoltage)
            {
                armRightMotor.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log)
            {
                log->Motor("lowerArm")
                .voltage(armRightMotor.GetMotorVoltage().GetValue())
                .position(armRightMotor.GetPosition().GetValue())
                .velocity(armRightMotor.GetVelocity().GetValue());
            }, this}};

    MotionMagicVoltage armVoltage
    { 0_tr };
};
