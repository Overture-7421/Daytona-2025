// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LowAlgae.h"
#include "Commands/ArmMotion/ArmMotion.h"

frc2::CommandPtr LowAlgae(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral,
                    frc2::cmd::RepeatingSequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                            elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                            ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae,
                                                    ArmConstants::WristLowAlgae, ElevatorConstants::LowAlgae).ToPtr(),
                                            intake->setIntakeCommand(IntakeConstants::AlgaeGrab,
                                                    IntakeConstants::JawAlgae)))).Until([intake] {
                        return intake->isAlgaeIn(IntakeConstants::JawAlgae);
                    })}, std::pair {SuperStructureStates::EnterHighAlgae,
                    frc2::cmd::RepeatingSequence(
                            frc2::cmd::Sequence(
                                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                            elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                            ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae,
                                                    ArmConstants::WristLowAlgae, ElevatorConstants::LowAlgae).ToPtr(),
                                            intake->setIntakeCommand(IntakeConstants::AlgaeGrab,
                                                    IntakeConstants::JawAlgae)))).Until([intake] {
                        return intake->isAlgaeIn(IntakeConstants::JawAlgae);
                    })}, std::pair {SuperStructureStates::HoldAlgae,
                    frc2::cmd::RepeatingSequence(
                            frc2::cmd::Sequence(
                                    /*
                                     frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                     ArmMotion(elevator, arm, ArmConstants::ArmAlgaeInter, ArmConstants::WristLowAlgae,
                                     ElevatorConstants::LowAlgae).ToPtr()),
                                     */
                                    frc2::cmd::Sequence(superStructure->setState(SuperStructureStates::EnterLowAlgae),
                                            elevator->setElevatorCommand(ElevatorConstants::LowAlgae),
                                            ArmMotion(elevator, arm, ArmConstants::ArmLowAlgae,
                                                    ArmConstants::WristLowAlgae, ElevatorConstants::LowAlgae).ToPtr(),
                                            intake->setIntakeCommand(IntakeConstants::AlgaeGrab,
                                                    IntakeConstants::JawAlgae)))).Until([intake] {
                        return intake->isAlgaeIn(IntakeConstants::JawAlgae);
                    })}

            ).AndThen(
                    frc2::cmd::Sequence(frc2::cmd::Wait(0.8_s),

                            frc2::cmd::Sequence(
                                    frc2::cmd::Parallel(
                                            intake->setIntakeCommand(IntakeConstants::AlgaeHold,
                                                    IntakeConstants::JawAlgae),
                                            superStructure->setState(SuperStructureStates::HoldAlgae)),
                                    frc2::cmd::Sequence(
                                            arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                            elevator->setElevatorCommand(ElevatorConstants::ClosedPosition)))));

}
