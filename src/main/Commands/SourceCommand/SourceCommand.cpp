// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceCommand.h"

frc2::CommandPtr SourceCommand(Arm* arm, Elevator* elevator, Intake* intake){
    return frc2::cmd::Parallel(
            frc2::cmd::Sequence(
        elevator->setElevatorCommand(ElevatorConstants::SourcePosition),
        frc2::cmd::WaitUntil([elevator]{
            return elevator->isElevatorAtPosition(ElevatorConstants::SourcePosition);
            
        }),

        arm->setArmCommand(Constants::ArmCoralStation, Constants::WristCoralStation),
        frc2::cmd::WaitUntil([arm]{
            return arm->isArmAtPosition(Constants::ArmCoralStation, Constants::WristCoralStation);
        })
        ),

        intake->moveIntake(IntakeConstants::CoralGrab)
    ).FinallyDo([=](){
        intake->moveIntake(IntakeConstants::StopIntake);
    });
}
