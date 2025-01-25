// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "HighAlgae.h"
#include "Commands/AlignToPose/AlignToPose.h"

frc::Pose2d pose {3.644_m, 2.976_m, 150.0_deg};

frc2::CommandPtr HighAlgae(Arm *arm, Elevator *elevator, Chassis *chassis) {
    return frc2::cmd::Parallel(
        frc2::cmd::Parallel(
            AlignToPose(chassis, pose).ToPtr(),
            elevator->setElevatorCommand(ElevatorConstants::HighAlgae),
        
            frc2::cmd::WaitUntil([elevator] {
                return elevator->isElevatorAtPosition(ElevatorConstants::HighAlgae);
            })
        ),
            arm->setArmCommand(ArmConstants::ArmHighAlgae, ArmConstants::WristHighAlgae), frc2::cmd::WaitUntil([arm] {
                return arm->isArmAtPosition(ArmConstants::ArmHighAlgae, ArmConstants::WristHighAlgae);

            })

    );
}

