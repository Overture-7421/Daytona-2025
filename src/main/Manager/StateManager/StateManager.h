// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "Manager/Transitions/Transitions.h"
#include "Manager/AlignManager/AlignManager.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Grabber/Grabber.h"
#include "Subsystems/Climber/Climber.h"
#include <vector>
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>
#include <OvertureLib/Gamepads/OverConsole/OverConsole.h>

class StateManager: public frc2::SubsystemBase {
public:
    StateManager(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber, Climber *climber,
            AlignManager *alignManager);

    void Periodic() override;

    Positions getStatePosition();

    // bool getExecute();
    // void setExecute(bool value);
    // frc2::CommandPtr setNewState();
    // frc2::CommandPtr setStatePosition();
    frc2::CommandPtr setStatePosition(Positions desiredState);
    frc2::CommandPtr setStateOverride();
    // int currentTransitionIndex = -1;

    Positions state = Positions::InitialPosition;

    frc2::CommandPtr InitialToSustained() { // Done
        return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest), intake->setState(Positions::InitialPosition),
                elevator->setState(Positions::SustainedPosition), arm->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition));
    }

    frc2::CommandPtr InitialToL4Front() { // Done
        return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),  intake->setState(Positions::L4Front),
                elevator->setState(Positions::L4Front),
            arm->setState(Positions::L4Front, Heading::Front),
                grabber->setState(Positions::L4Front)).BeforeStarting(
                setStatePosition(Positions::L4Front));
    }

    frc2::CommandPtr InitialToL4Back() { // Done
        return frc2::cmd::Parallel(climber->setClimberCommand(ClimberConstants::ClimberRest), arm->setState(Positions::L4Back, Heading::Back),
                elevator->setState(Positions::L4Back), intake->setState(Positions::L4Back),
                grabber->setState(Positions::L4Back)).BeforeStarting(
                setStatePosition(Positions::L4Back));
    }

    frc2::CommandPtr SustainedToIntake() { // Done
        return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest), intake->setState(Positions::Intake), elevator->setState(Positions::Intake),
                arm->setState(Positions::Intake), grabber->setState(Positions::Intake)).OnlyIf([this] {
            return !intake->isCoralIn();
        }).BeforeStarting(setStatePosition(Positions::Intake)).Until([this] {
            return intake->isCoralIn();
        });
    }

    frc2::CommandPtr SustainedToAlgaeLowReef() { // Done
        return frc2::cmd::Sequence(
                frc2::cmd::Parallel(intake->setState(Positions::AlgaeLowReef), arm->setState(Positions::AlgaeLowReef)),
                elevator->setState(Positions::AlgaeLowReef), grabber->setState(Positions::AlgaeLowReef)).BeforeStarting(setStatePosition(Positions::AlgaeLowReef)).OnlyIf(
                [this] {
                    return !intake->isCoralIn();
                });
    }

    frc2::CommandPtr SustainedToAlgaeHighReef() { // Done
        return frc2::cmd::Parallel(intake->setState(Positions::AlgaeHighReef), arm->setState(Positions::AlgaeHighReef),
                elevator->setState(Positions::AlgaeHighReef), grabber->setState(Positions::AlgaeHighReef)).BeforeStarting(setStatePosition(Positions::AlgaeHighReef)).OnlyIf(
                [this] {
                    return !intake->isCoralIn();
                });
    }

    frc2::CommandPtr SustainedToAlgaeGround() { // Done
        return frc2::cmd::Sequence(intake->setState(Positions::AlgaeGround), arm->setState(Positions::AlgaeGround),
                elevator->setState(Positions::AlgaeGround), grabber->setState(Positions::AlgaeGround)).BeforeStarting(setStatePosition(Positions::AlgaeGround)).OnlyIf(
                [this] {
                    return !intake->isCoralIn();
                });
    }

    frc2::CommandPtr SustainedToEndPosition() { // Done
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),  climber->setClimberCommand(ClimberConstants::ClimberOpen)).BeforeStarting(setStatePosition(Positions::EndPosition));
    }

    frc2::CommandPtr IntakeToSustained() { // Done
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !intake->isCoralIn();
        });
    }

    frc2::CommandPtr IntakeToL1Position() { // Done
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
                frc2::cmd::Sequence(intake->setStateRollers(Positions::Through),
                        intake->setStateIntake(Positions::L1Position), intake->setState(Positions::L1Position))).BeforeStarting(
                setStatePosition(Positions::L1Position)).OnlyIf([this] {
            return intake->isCoralIn();
        });
    }

    frc2::CommandPtr AlgaeHighReefToSustained() { // Done
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !grabber->isAlgaeIn() && !intake->isCoralIn();
        });
    }

    frc2::CommandPtr AlgaeHighReefToAlgaeHold() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
                intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold)).BeforeStarting(setStatePosition(Positions::AlgaeHold)).OnlyIf(
                [this] {
                    return grabber->isAlgaeIn() && !intake->isCoralIn();
                });
    }

    frc2::CommandPtr AlgaeLowReefToSustained() { // Done
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !grabber->isAlgaeIn() && !intake->isCoralIn();
        });
    }

    frc2::CommandPtr AlgaeLowReefToAlgaeHold() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
                intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold)).BeforeStarting(setStatePosition(Positions::AlgaeHold)).OnlyIf(
                [this] {
                    return grabber->isAlgaeIn() && !intake->isCoralIn();
                });
    }

    frc2::CommandPtr AlgaeGroundToSustained() { // Done
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !grabber->isAlgaeIn() && !intake->isCoralIn();
        });
    }

    frc2::CommandPtr AlgaeGroundToAlgaeHold() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
                intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold)).BeforeStarting(setStatePosition(Positions::AlgaeHold)).OnlyIf(
                [this] {
                    return grabber->isAlgaeIn() && !intake->isCoralIn();
                });
    }

    frc2::CommandPtr L1PositionToL1Confirm() { // Done
        return frc2::cmd::Parallel(elevator->setState(Positions::L1Confirm), arm->setState(Positions::L1Confirm),
                intake->setState(Positions::L1Confirm), grabber->setState(Positions::L1Confirm)).BeforeStarting(setStatePosition(Positions::L1Confirm));
    }

    frc2::CommandPtr L1PositionToCoralHold() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::CoralHold), intake->setStateRollers(Positions::Through),
                intake->setStateIntake(Positions::CoralHold), grabber->setState(Positions::CoralHold),

                elevator->setState(Positions::CoralHold), intake->setState(Positions::CoralHold)).BeforeStarting(setStatePosition(Positions::SustainedPosition)).OnlyIf(
                [this] {
                    return !grabber->isAlgaeIn() && intake->isCoralIn();
                });
    }

    frc2::CommandPtr L1ConfirmToSustained() { // Done
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !intake->isCoralIn();
        });
    }

    frc2::CommandPtr CoralHoldToL1Position() { // Done
        return frc2::cmd::Sequence(
                frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::CoralHold)),
                frc2::cmd::Parallel(intake->setState(Positions::SustainToL1), grabber->setState(Positions::CoralSpit))).BeforeStarting(setStatePosition(Positions::Intake));
    }

    frc2::CommandPtr CoralHoldToL2Front() { // Done
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L2Front, Heading::Front),
                        elevator->setState(Positions::L2Front)), intake->setState(Positions::L2Front),
                grabber->setState(Positions::L2Front)).BeforeStarting(
                setStatePosition(Positions::L2Front));
    }

    frc2::CommandPtr CoralHoldToL3Front() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L3Front, Heading::Front),
                elevator->setState(Positions::L3Front), intake->setState(Positions::L3Front),
                grabber->setState(Positions::L3Front)).BeforeStarting(
                setStatePosition(Positions::L3Front));
    }

    frc2::CommandPtr CoralHoldToL4Front() {  // Done
        return frc2::cmd::Sequence(arm->setState(Positions::L4Front, Heading::Front),
                elevator->setState(Positions::L4Front), intake->setState(Positions::L4Front),
                grabber->setState(Positions::L4Front)).BeforeStarting(
                setStatePosition(Positions::L4Front));
    }

    frc2::CommandPtr CoralHoldToL2Back() { // Done
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L2Back, Heading::Back),
                        elevator->setState(Positions::L2Back)), intake->setState(Positions::L2Back),
                grabber->setState(Positions::L2Back)).BeforeStarting(
                setStatePosition(Positions::L2Back));
    }

    frc2::CommandPtr CoralHoldToL3Back() {  // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L3Back, Heading::Back),
                elevator->setState(Positions::L3Back), intake->setState(Positions::L3Back),
                grabber->setState(Positions::L3Back)).BeforeStarting(
                setStatePosition(Positions::L3Back));
    }

    frc2::CommandPtr CoralHoldToL4Back() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L4Back, Heading::Back),
                elevator->setState(Positions::L4Back), intake->setState(Positions::L4Back),
                grabber->setState(Positions::L4Back)).BeforeStarting(
                setStatePosition(Positions::L4Back));
    }

    frc2::CommandPtr CoralHoldToEndPosition() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition)).BeforeStarting(setStatePosition(Positions::EndPosition));
    }

    frc2::CommandPtr AlgaeHoldToNet() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::NetPosition), elevator->setState(Positions::NetPosition),
                grabber->setState(Positions::NetPosition), intake->setState(Positions::NetPosition)).BeforeStarting(setStatePosition(Positions::NetPosition)).OnlyIf(
                [this] {
                    return grabber->isAlgaeIn();
                });
    }

    frc2::CommandPtr AlgaeHoldToProcessor() { // Done
        return frc2::cmd::Sequence(intake->setState(Positions::ProcessorPosition),
                arm->setState(Positions::ProcessorPosition), elevator->setState(Positions::ProcessorPosition),
                grabber->setState(Positions::ProcessorPosition)).BeforeStarting(
                setStatePosition(Positions::ProcessorPosition)).OnlyIf([this] {
            return grabber->isAlgaeIn();
        });
    }

    frc2::CommandPtr AlgaeHoldToEndPosition() { // Done
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition), climber->setClimberCommand(ClimberConstants::ClimberOpen)).BeforeStarting(setStatePosition(Positions::EndPosition));
    }

    frc2::CommandPtr L2FrontToFrontConfirm() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L2FrontConfirm, Heading::Front),
                elevator->setState(Positions::L2FrontConfirm), intake->setState(Positions::L2FrontConfirm),
                grabber->setState(Positions::L2FrontConfirm)).BeforeStarting(
                setStatePosition(Positions::L2FrontConfirm));
    }

    frc2::CommandPtr L3FrontToFrontConfirm() {  // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L3FrontConfirm, Heading::Front),
                elevator->setState(Positions::L3FrontConfirm), intake->setState(Positions::L3FrontConfirm),
                grabber->setState(Positions::L3FrontConfirm)).BeforeStarting(
                setStatePosition(Positions::L3FrontConfirm));
    }

    frc2::CommandPtr L4FrontToFrontConfirm() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L4FrontConfirm, Heading::Front),
                elevator->setState(Positions::L4FrontConfirm), intake->setState(Positions::L4FrontConfirm),
                grabber->setState(Positions::L4FrontConfirm)).BeforeStarting(
                setStatePosition(Positions::L4FrontConfirm));
    }

    frc2::CommandPtr L2BackToBackConfirm() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L2BackConfirm, Heading::Back),
                elevator->setState(Positions::L2BackConfirm), intake->setState(Positions::L2BackConfirm),
                grabber->setState(Positions::L2BackConfirm)).BeforeStarting(
                setStatePosition(Positions::L2BackConfirm));
    }

    frc2::CommandPtr L3BackToBackConfirm() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L3BackConfirm, Heading::Back),
                elevator->setState(Positions::L3BackConfirm), intake->setState(Positions::L3BackConfirm),
                grabber->setState(Positions::L3BackConfirm)).BeforeStarting(
                setStatePosition(Positions::L3BackConfirm));
    }

    frc2::CommandPtr L4BackToBackConfirm() { // Done
        return frc2::cmd::Parallel(arm->setState(Positions::L4BackConfirm, Heading::Back),
                elevator->setState(Positions::L4BackConfirm), intake->setState(Positions::L4BackConfirm),
                grabber->setState(Positions::L4BackConfirm)).BeforeStarting(
                setStatePosition(Positions::L4BackConfirm));
    }

    frc2::CommandPtr NetPositionToNetConfirm() { // Done
        return frc2::cmd::Sequence(grabber->setState(Positions::AlgaeTension), grabber->setState(Positions::NetConfirm),
                arm->setState(Positions::NetConfirm), elevator->setState(Positions::NetConfirm),
                intake->setState(Positions::NetConfirm)).BeforeStarting(
                setStatePosition(Positions::NetConfirm)).OnlyIf([this] {
            return grabber->isAlgaeIn();
        });
    }

    frc2::CommandPtr ProcessorPositionToProcessorConfirm() { // Done
        return frc2::cmd::Sequence(grabber->setState(Positions::ProcessorConfirm),
                intake->setState(Positions::ProcessorConfirm), arm->setState(Positions::ProcessorConfirm),
                elevator->setState(Positions::ProcessorConfirm)).BeforeStarting(
                setStatePosition(Positions::ProcessorConfirm)).OnlyIf([this] {
            return grabber->isAlgaeIn();
        });
    }

    frc2::CommandPtr FrontConfirmToSustained() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition));
    }

    frc2::CommandPtr BackConfirmToSustained() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition));
    }

    frc2::CommandPtr NetConfirmToSustained() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !grabber->isAlgaeIn() && !intake->isCoralIn();
        });
    }

    frc2::CommandPtr ProcessorConfirmToSustained() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition)).BeforeStarting(
                setStatePosition(Positions::SustainedPosition)).OnlyIf([this] {
            return !grabber->isAlgaeIn() && !intake->isCoralIn();
        });
    }

    frc2::CommandPtr EndToInitial() {
        return frc2::cmd::Sequence(intake->setState(Positions::InitialPosition),
                arm->setState(Positions::InitialPosition), elevator->setState(Positions::InitialPosition),
                grabber->setState(Positions::InitialPosition));
    }

private:
    // bool execute = false;
    Positions desiredState = Positions::InitialPosition;

    Intake *intake = nullptr;
    Arm *arm = nullptr;
    Elevator *elevator = nullptr;
    Grabber *grabber = nullptr;
    Climber *climber = nullptr;
    AlignManager *alignManager = nullptr;

};
