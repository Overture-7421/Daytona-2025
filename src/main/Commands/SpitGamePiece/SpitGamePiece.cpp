// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SpitGamePiece.h"

frc2::CommandPtr SpitGamePiece(Intake *intake, SuperStructure *superStructure, Elevator *elevator, Arm *arm) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral,
                    frc2::cmd::Sequence(
                            ArmMotion(elevator, arm, ArmConstants::ArmScore, ArmConstants::WristClosed,
                                    elevator->getPosition()).ToPtr(),
                            intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralOpen),
                            superStructure->setState(SuperStructureStates::SpitCoral))}, std::pair {
                    SuperStructureStates::HoldAlgae,
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(
                                    ArmMotion(elevator, arm, 30_deg, ArmConstants::WristClosed, elevator->getPosition()).ToPtr().BeforeStarting(
                                            [arm] {
                                                arm->changeArmSpeeds(ArmConstants::SlowVelocity,
                                                        ArmConstants::SlowAccelerationa);
                                            }
                                    ).FinallyDo(
                                            [arm] {
                                                arm->changeArmSpeeds(ArmConstants::ArmCruiseVelocity,
                                                        ArmConstants::ArmCruiseAcceleration);
                                            }
                                    ),
                                    intake->setIntakeCommand(IntakeConstants::AlgaeHold, IntakeConstants::JawAlgae)),
                            frc2::cmd::Wait(0.2_s),
                            //intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawAlgaeSpit),
                            intake->setIntakeCommand(IntakeConstants::AlgaeRelease, IntakeConstants::JawAlgaeSpit),
                            superStructure->setState(SuperStructureStates::SpitAlgae))}

            );

}

frc2::CommandPtr SpitGamePieceAuto(Intake *intake, SuperStructure *superStructure, Elevator *elevator, Arm *arm) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral,
                    frc2::cmd::Sequence(
                            ArmMotion(elevator, arm, ArmConstants::ArmScore, ArmConstants::WristClosed,
                                    elevator->getPosition()).ToPtr().WithTimeout(1.5_s),
                            intake->setIntakeCommand(IntakeConstants::StopIntake, IntakeConstants::JawCoralOpen),
                            superStructure->setState(SuperStructureStates::SpitCoral))}, std::pair {
                    SuperStructureStates::HoldAlgae, frc2::cmd::Parallel(
                            intake->setIntakeCommand(IntakeConstants::AlgaeRelease, IntakeConstants::JawAlgae),
                            superStructure->setState(SuperStructureStates::SpitAlgae))});

}
