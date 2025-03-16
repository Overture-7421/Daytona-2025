// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SourceCommand.h"
#include "Commands/ArmMotion/ArmMotion.h"

int nextButton = frc::XboxController::Button::kRightBumper;
int nextConsoleButton = 6;

frc2::CommandPtr SourceCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure,
        OverXboxController *gamepad) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterCoralGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterAlgaeGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation)

                            )).Until([intake] {
                return intake->isCoralIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldCoral)),
                            WaitForButton(gamepad, nextButton),
                            frc2::cmd::Sequence(
                                    arm->setArmCommand(ArmConstants::ArmCoralStationAway, ArmConstants::WristClosed),
                                    frc2::cmd::Wait(0.5_s),
                                    arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))

                                    ));

}

frc2::CommandPtr SourceCommand(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure,
        frc2::CommandGenericHID *gamepadConsole) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterCoralGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterAlgaeGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation)

                            )).Until([intake] {
                return intake->isCoralIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldCoral)),
                            WaitForButtonConsole(gamepadConsole, nextConsoleButton),
                            frc2::cmd::Sequence(
                                    arm->setArmCommand(ArmConstants::ArmCoralStationAway, ArmConstants::WristClosed),
                                    frc2::cmd::Wait(0.5_s),
                                    arm->setArmCommand(ArmConstants::ArmClosed, ArmConstants::WristClosed),
                                    elevator->setElevatorCommand(ElevatorConstants::ClosedPosition))

                                    ));

}

frc2::CommandPtr SourceCommandAuto(Arm *arm, Elevator *elevator, Intake *intake, SuperStructure *superStructure) {
    return frc2::cmd::Select < SuperStructureStates
            > ([superStructure] {
                return superStructure->getState();
            },
            std::pair {SuperStructureStates::HoldCoral, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterCoralGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation))).Until([intake] {
                return intake->isCoralIn();
            })}, std::pair {SuperStructureStates::EnterAlgaeGround, frc2::cmd::RepeatingSequence(
                    frc2::cmd::Parallel(elevator->setElevatorCommand(ElevatorConstants::CoralStationPosition),
                            ArmMotion(elevator, arm, ArmConstants::ArmCoralStation, ArmConstants::WristCoralStation,
                                    ElevatorConstants::CoralStationPosition).ToPtr(),
                            intake->moveIntake(IntakeConstants::CoralGrab),
                            superStructure->setState(SuperStructureStates::EnterCoralStation)

                            )).Until([intake] {
                return intake->isCoralIn();
            })}

            ).AndThen(
                    frc2::cmd::Sequence(
                            frc2::cmd::Parallel(intake->moveIntake(IntakeConstants::StopIntake),
                                    superStructure->setState(SuperStructureStates::HoldCoral))
                                    ));

}
