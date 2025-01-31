// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr SourceCommand(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),

            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                    ElevatorConstants::CoralStationPosition).ToPtr(),

            intake->moveIntake(IntakeConstants::CoralGrab).FinallyDo([=]() {
                intake->moveIntake(IntakeConstants::StopIntake);
            }));
}
