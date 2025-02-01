// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpitGamePiece.h"

frc2::CommandPtr SpitGamePiece(Intake *intake) {
    return frc2::cmd::Select < IntakeStates
            > ([intake] {
                return intake->getState();
            },
            std::pair {IntakeStates::HoldCoral, frc2::cmd::Parallel(
                    intake->setIntakeCommand(IntakeConstants::CoralRelease, IntakeConstants::JawCoralOpen, IntakeStates::SpitCoral))}, std::pair {
                    IntakeStates::HoldAlgae, frc2::cmd::Parallel(
                            intake->setIntakeCommand(IntakeConstants::AlgaeRelease, IntakeConstants::JawAlgae, IntakeStates::SpitAlgae))}

            );

}
