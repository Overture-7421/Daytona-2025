// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StateManager.h"
#include <frc/smartdashboard/SmartDashboard.h>

StateManager::StateManager(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber, Climber *climber) {
    this->intake = intake;
    this->arm = arm;
    this->elevator = elevator;
    this->grabber = grabber;
    this->climber = climber;
}

Positions StateManager::getStatePosition() {
    return state;
}

frc2::CommandPtr StateManager::setStatePosition(Positions desiredState) {
    return frc2::cmd::RunOnce([this, desiredState] {
        this->state = desiredState;
    });
}

frc2::CommandPtr StateManager::setStateOverride() {
    return frc2::cmd::RunOnce([this] {
        this->state = Positions::SustainedPosition;
    });
}

void StateManager::Periodic() {
    frc::SmartDashboard::PutString("StateManager/CurrentState", std::to_string(static_cast<int>(state)));
}

frc2::CommandPtr StateManager::InitialToSustained() {
    return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),
            intake->setState(Positions::InitialPosition), elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), grabber->setState(Positions::SustainedPosition)).AlongWith(
            setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::InitialToL4Front() {
    return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),
            intake->setState(Positions::L1Position), elevator->setState(Positions::L4Front),
            arm->setState(Positions::L4FrontAuto, Heading::Front), grabber->setState(Positions::L4Front)).AlongWith(
            setStatePosition(Positions::L4FrontAuto));
}

frc2::CommandPtr StateManager::InitialToL4Back() {
    return frc2::cmd::Parallel(climber->setClimberCommand(ClimberConstants::ClimberRest),
            intake->setState(Positions::L1Position), elevator->setState(Positions::L4Back),
            arm->setState(Positions::L4BackAuto, Heading::Back), grabber->setState(Positions::L4Back)).AlongWith(
            setStatePosition(Positions::L4BackAuto));
}

frc2::CommandPtr StateManager::SustainedToIntake() {
    return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),
            frc2::cmd::Parallel(intake->setState(Positions::Intake), elevator->setState(Positions::Intake)),
            arm->setState(Positions::Intake), grabber->setState(Positions::Intake)).AlongWith(
            setStatePosition(Positions::Intake));
}

frc2::CommandPtr StateManager::SustainedToAlgaeLowReef() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(intake->setState(Positions::AlgaeLowReef), arm->setState(Positions::AlgaeLowReef)),
            elevator->setState(Positions::AlgaeLowReef), grabber->setState(Positions::AlgaeLowReef)).AlongWith(
            setStatePosition(Positions::AlgaeLowReef));
}

frc2::CommandPtr StateManager::SustainedToAlgaeHighReef() {
    return frc2::cmd::Parallel(intake->setState(Positions::AlgaeHighReef), arm->setState(Positions::AlgaeHighReef),
            elevator->setState(Positions::AlgaeHighReef), grabber->setState(Positions::AlgaeHighReef)).AlongWith(
            setStatePosition(Positions::AlgaeHighReef));
}

frc2::CommandPtr StateManager::SustainedToAlgaeGround() {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            frc2::cmd::Parallel(intake->setState(Positions::AlgaeGround), arm->setState(Positions::AlgaeGround)),
            elevator->setState(Positions::AlgaeGround), grabber->setState(Positions::AlgaeGround)).AlongWith(
            setStatePosition(Positions::AlgaeGround));
}

frc2::CommandPtr StateManager::AlgaeHoldToAlgaeGround() {
    return frc2::cmd::Sequence(elevator->setState(Positions::AlgaeHold),
            frc2::cmd::Parallel(intake->setState(Positions::AlgaeGround), arm->setState(Positions::AlgaeGround)),
            elevator->setState(Positions::AlgaeGround), grabber->setState(Positions::AlgaeGround)).AlongWith(
            setStatePosition(Positions::AlgaeGround));
}

frc2::CommandPtr StateManager::SustainedToEndPosition() {
    return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
            elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
            climber->setClimberCommand(ClimberConstants::ClimberOpen)).AlongWith(
            setStatePosition(Positions::EndPosition));
}

frc2::CommandPtr StateManager::IntakeToL1Position() {
    return frc2::cmd::Parallel(
            frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
            frc2::cmd::Sequence(intake->setStateRollers(Positions::Through),
                    intake->setStateIntake(Positions::L1Position), intake->setState(Positions::L1Position))).AlongWith(
            setStatePosition(Positions::L1Position));
}

frc2::CommandPtr StateManager::AlgaeHighReefToSustained() {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::AlgaeHighReefToAlgaeHold() {
    return frc2::cmd::Sequence(grabber->setState(Positions::AlgaeHold), arm->setState(Positions::AlgaeHold),
            elevator->setState(Positions::AlgaeHold), intake->setState(Positions::AlgaeHold)).AlongWith(
            setStatePosition(Positions::AlgaeHold)).BeforeStarting(frc2::cmd::RunOnce([this] {
        return arm->setArmLowerSpeed();
    })).AndThen(frc2::cmd::RunOnce([this] {
        return arm->setArmNormalSpeed();
    }));
}

frc2::CommandPtr StateManager::AlgaeLowReefToSustained() {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::AlgaeLowReefToAlgaeHold() {
    return frc2::cmd::Sequence(grabber->setState(Positions::AlgaeHold), arm->setState(Positions::AlgaeHold),
            elevator->setState(Positions::AlgaeHold), intake->setState(Positions::AlgaeHold)).AlongWith(
            setStatePosition(Positions::AlgaeHold)).BeforeStarting(frc2::cmd::RunOnce([this] {
        return arm->setArmLowerSpeed();
    })).AndThen(frc2::cmd::RunOnce([this] {
        return arm->setArmNormalSpeed();
    }));
}

frc2::CommandPtr StateManager::AlgaeGroundToSustained() {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::AlgaeGroundToAlgaeHold() {
    return frc2::cmd::Sequence(grabber->setState(Positions::AlgaeHold), arm->setState(Positions::AlgaeHold),
            elevator->setState(Positions::AlgaeHold), intake->setState(Positions::AlgaeHold)).AlongWith(
            setStatePosition(Positions::AlgaeHold)).BeforeStarting(frc2::cmd::RunOnce([this] {
        return arm->setArmLowerSpeed();
    })).AndThen(frc2::cmd::RunOnce([this] {
        return arm->setArmNormalSpeed();
    }));
}

frc2::CommandPtr StateManager::L1PositionToL1Confirm() {
    return frc2::cmd::Parallel(elevator->setState(Positions::L1Confirm), arm->setState(Positions::L1Confirm),
            intake->setState(Positions::L1Confirm), grabber->setState(Positions::L1Confirm)).AlongWith(
            setStatePosition(Positions::L1Confirm));
}

frc2::CommandPtr StateManager::IntakeToCoralHold() {
    return frc2::cmd::Sequence(arm->setState(Positions::CoralHold), intake->setStateRollers(Positions::Through),
            intake->setStateIntake(Positions::CoralHold), grabber->setState(Positions::CoralHold),
            elevator->setState(Positions::CoralHold), intake->setState(Positions::CoralHold), frc2::cmd::Wait(0.2_s),
            grabber->setState(Positions::InitialPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::L1ClosedToCoralHold() {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition), arm->setState(Positions::CoralHold),
            intake->setStateIntake(Positions::CoralHold), elevator->setState(Positions::CoralHold),
            grabber->setState(Positions::CoralHold), intake->setStateRollers(Positions::Through),
            intake->setState(Positions::CoralHold), frc2::cmd::Wait(0.2_s),
            grabber->setState(Positions::InitialPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::L1PositionToCoralHoldAuto() {
    return (frc2::cmd::Sequence(arm->setState(Positions::CoralHold), intake->setStateRollers(Positions::Through),
            intake->setStateIntake(Positions::CoralHold), grabber->setState(Positions::CoralHold),
            elevator->setState(Positions::CoralHold), intake->setState(Positions::CoralHold), frc2::cmd::Wait(0.2_s),
            grabber->setState(Positions::InitialPosition)).AlongWith(setStatePosition(Positions::SustainedPosition)));
}

frc2::CommandPtr StateManager::L1ConfirmToSustained() {
    return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition)));
}

frc2::CommandPtr StateManager::CoralHoldToL1Position() {
    return (frc2::cmd::Sequence(elevator->setState(Positions::CoralHold), arm->setState(Positions::L1Position),
            intake->setState(Positions::SustainToL1), grabber->setState(Positions::CoralSpit),
            elevator->setState(Positions::L1Position), frc2::cmd::Wait(0.2_s), intake->setState(Positions::L1Position)).AlongWith(
            setStatePosition(Positions::L1Position)));
}

frc2::CommandPtr StateManager::CoralHoldToL2Front() {
    return frc2::cmd::Parallel(
            frc2::cmd::Sequence(arm->setState(Positions::FrontTransition, Heading::Front),
                    elevator->setState(Positions::L2Front), arm->setState(Positions::L2Front, Heading::Front),
                    intake->setState(Positions::L2Front)), grabber->setState(Positions::L2Front)).AlongWith(
            setStatePosition(Positions::L2Front));
}

frc2::CommandPtr StateManager::CoralHoldToL3Front() {
    return frc2::cmd::Sequence(arm->setState(Positions::L3Front, Heading::Front),
            elevator->setState(Positions::L3Front), intake->setState(Positions::L3Front),
            grabber->setState(Positions::L3Front)).AlongWith(setStatePosition(Positions::L3Front));
}

frc2::CommandPtr StateManager::CoralHoldToL4Front() {
    return frc2::cmd::Sequence(arm->setState(Positions::L4Front, Heading::Front),
            elevator->setState(Positions::L4Front), intake->setState(Positions::L4Front),
            grabber->setState(Positions::L4Front)).AlongWith(setStatePosition(Positions::L4Front));
}

frc2::CommandPtr StateManager::CoralHoldToL2Back() {
    return frc2::cmd::Parallel(
            frc2::cmd::Sequence(arm->setState(Positions::BackTransition, Heading::Back),
                    elevator->setState(Positions::L2Back), arm->setState(Positions::L2Back, Heading::Back),
                    intake->setState(Positions::L2Back)), grabber->setState(Positions::L2Back)).AlongWith(
            setStatePosition(Positions::L2Back));
}

frc2::CommandPtr StateManager::CoralHoldToL3Back() {
    return frc2::cmd::Sequence(arm->setState(Positions::L3Back, Heading::Back), elevator->setState(Positions::L3Back),
            intake->setState(Positions::L3Back), grabber->setState(Positions::L3Back)).AlongWith(
            setStatePosition(Positions::L3Back));
}

frc2::CommandPtr StateManager::CoralHoldToL4Back() {
    return frc2::cmd::Sequence(arm->setState(Positions::L4Back, Heading::Back), elevator->setState(Positions::L4Back),
            intake->setState(Positions::L4Back), grabber->setState(Positions::L4Back)).AlongWith(
            setStatePosition(Positions::L4Back));
}

frc2::CommandPtr StateManager::ReefFrontToReefPosition(Positions reefPosition) {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::FrontTransition, Heading::Front),
                    elevator->setState(reefPosition)), arm->setState(reefPosition, Heading::Front),
            intake->setState(reefPosition), grabber->setState(reefPosition)).AlongWith(setStatePosition(reefPosition));
}

frc2::CommandPtr StateManager::ReefBackToReefPosition(Positions reefPosition) {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::BackTransition, Heading::Back),
                    elevator->setState(reefPosition)), arm->setState(reefPosition, Heading::Back),
            intake->setState(reefPosition), grabber->setState(reefPosition)).AlongWith(setStatePosition(reefPosition));
}

frc2::CommandPtr StateManager::AlgaeHoldToNet() {
    return (frc2::cmd::Sequence(arm->setState(Positions::NetPosition), elevator->setState(Positions::NetPosition),
            grabber->setState(Positions::NetPosition), intake->setState(Positions::NetPosition)).AlongWith(
            setStatePosition(Positions::NetPosition)));
}

frc2::CommandPtr StateManager::AlgaeHoldToProcessor() {
    return (frc2::cmd::Sequence(intake->setState(Positions::ProcessorPosition),
            elevator->setState(Positions::ProcessorPosition), arm->setState(Positions::ProcessorPosition),
            grabber->setState(Positions::ProcessorPosition)).AlongWith(setStatePosition(Positions::ProcessorPosition)));
}

frc2::CommandPtr StateManager::L2FrontToFrontConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(elevator->setState(Positions::L2FrontConfirm),
                    intake->setState(Positions::L2FrontConfirm),
                    arm->setState(Positions::L2FrontConfirm, Heading::Front),
                    grabber->setState(Positions::L2FrontConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::FrontConfirm));
}

frc2::CommandPtr StateManager::L3FrontToFrontConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(elevator->setState(Positions::L3FrontConfirm),
                    intake->setState(Positions::L3FrontConfirm),
                    arm->setState(Positions::L3FrontConfirm, Heading::Front),
                    grabber->setState(Positions::L3FrontConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::FrontConfirm));
}

frc2::CommandPtr StateManager::L4FrontToFrontConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(elevator->setState(Positions::L4FrontConfirm),
                    intake->setState(Positions::L4FrontConfirm),
                    arm->setState(Positions::L4FrontConfirm, Heading::Front),
                    grabber->setState(Positions::L4FrontConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::FrontConfirm));
}

frc2::CommandPtr StateManager::L4FrontAutoToFrontAutoConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::L4FrontAutoConfirm, Heading::Front),
                    elevator->setState(Positions::L4FrontConfirm), intake->setState(Positions::L4FrontConfirm),
                    grabber->setState(Positions::L4FrontConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::L4FrontAutoConfirm));
}

frc2::CommandPtr StateManager::L4BackAutoToFrontAutoConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::L4BackAutoConfirm, Heading::Back),
                    elevator->setState(Positions::L4BackConfirm), intake->setState(Positions::L4BackConfirm),
                    grabber->setState(Positions::L4BackConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::L4BackAutoConfirm));
}

frc2::CommandPtr StateManager::L2BackToBackConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::L2BackConfirm, Heading::Back),
                    elevator->setState(Positions::L2BackConfirm), intake->setState(Positions::L2BackConfirm),
                    grabber->setState(Positions::L2BackConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::BackConfirm));
}

frc2::CommandPtr StateManager::L3BackToBackConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::L3BackConfirm, Heading::Back),
                    elevator->setState(Positions::L3BackConfirm), intake->setState(Positions::L3BackConfirm),
                    grabber->setState(Positions::L3BackConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::BackConfirm));
}

frc2::CommandPtr StateManager::L4BackToBackConfirm() {
    return frc2::cmd::Sequence(
            frc2::cmd::Parallel(arm->setState(Positions::L4BackConfirm, Heading::Back),
                    elevator->setState(Positions::L4BackConfirm), intake->setState(Positions::L4BackConfirm),
                    grabber->setState(Positions::L4BackConfirm)), grabber->setState(Positions::CoralSpit)).AlongWith(
            setStatePosition(Positions::BackConfirm));
}

frc2::CommandPtr StateManager::NetPositionToNetConfirm() {
    return (frc2::cmd::Sequence(
            frc2::cmd::Parallel(frc2::cmd::Sequence(frc2::cmd::Wait(0.12_s), grabber->setState(Positions::NetConfirm)),
                    arm->setState(Positions::NetConfirm)), elevator->setState(Positions::NetConfirm),
            intake->setState(Positions::NetConfirm)).AlongWith(setStatePosition(Positions::NetConfirm)));
}

frc2::CommandPtr StateManager::ProcessorPositionToProcessorConfirm() {
    return (frc2::cmd::Sequence(grabber->setState(Positions::ProcessorConfirm),
            intake->setState(Positions::ProcessorConfirm), arm->setState(Positions::ProcessorConfirm),
            elevator->setState(Positions::ProcessorConfirm)).AlongWith(setStatePosition(Positions::ProcessorConfirm)));
}

frc2::CommandPtr StateManager::FrontConfirmToSustained() {
    return frc2::cmd::Sequence(grabber->setState(Positions::SustainedPosition),
            elevator->setState(Positions::SustainedPosition), arm->setState(Positions::SustainedPosition),
            intake->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::BackConfirmToSustained() {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition));
}

frc2::CommandPtr StateManager::NetConfirmToSustained() {
    return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition)));
}

frc2::CommandPtr StateManager::ProcessorConfirmToSustained() {
    return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            grabber->setState(Positions::SustainedPosition)).AlongWith(setStatePosition(Positions::SustainedPosition)));
}

frc2::CommandPtr StateManager::AllToInitial() {
    return frc2::cmd::Sequence(intake->setState(Positions::InitialPosition), arm->setState(Positions::InitialPosition),
            elevator->setState(Positions::InitialPosition), grabber->setState(Positions::InitialPosition)).AlongWith(
            setStatePosition(Positions::InitialPosition));
}

frc2::CommandPtr StateManager::L1PositionClosed() {
    return frc2::cmd::Sequence(intake->setState(Positions::AlgaeHold), arm->setState(Positions::AlgaeHold),
            grabber->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold)).AlongWith(
            setStatePosition(Positions::AlgaeHold));
}

frc2::CommandPtr StateManager::L1PositionIntake() {
    return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),
            intake->setState(Positions::Intake), arm->setState(Positions::AlgaeHold),
            elevator->setState(Positions::AlgaeHold), grabber->setState(Positions::Intake)).AlongWith(
            setStatePosition(Positions::Intake));
}

frc2::CommandPtr StateManager::L1ClosedConfirm() {
    return intake->setState(Positions::L1Confirm);
}
