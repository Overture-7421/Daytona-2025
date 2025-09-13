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

class StateManager : public frc2::SubsystemBase {
public:
	StateManager(Intake* intake, Arm* arm, Elevator* elevator, Grabber* grabber, Climber* climber,
		AlignManager* alignManager);

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

	frc2::CommandPtr InitialToSustained() {
		return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),
			intake->setState(Positions::InitialPosition), elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition));
	}

	frc2::CommandPtr InitialToL4Front() {
		return frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest), intake->setState(Positions::L4Front),
			elevator->setState(Positions::L4Front),
			arm->setState(Positions::L4FrontAuto, Heading::Front),
			grabber->setState(Positions::L4Front)).AlongWith(
				setStatePosition(Positions::L4FrontAuto));
	}

	frc2::CommandPtr InitialToL4Back() {
		return frc2::cmd::Parallel(climber->setClimberCommand(ClimberConstants::ClimberRest),
			arm->setState(Positions::L4Back, Heading::Back), elevator->setState(Positions::L4Back),
			intake->setState(Positions::L4Back), grabber->setState(Positions::L4Back)).AlongWith(
				setStatePosition(Positions::L4Back));
	}

	frc2::CommandPtr SustainedToIntake() {
		return (frc2::cmd::Sequence(climber->setClimberCommand(ClimberConstants::ClimberRest),
			intake->setState(Positions::Intake), elevator->setState(Positions::Intake),
			arm->setState(Positions::Intake), grabber->setState(Positions::Intake))).OnlyIf([this] {
			return !intake->isCoralIn();
		}).AlongWith(setStatePosition(Positions::Intake)).Until([this] {
			return intake->isCoralIn();
		});
	}

	frc2::CommandPtr SustainedToAlgaeLowReef() {
		return (frc2::cmd::Sequence(
			frc2::cmd::Parallel(intake->setState(Positions::AlgaeLowReef), arm->setState(Positions::AlgaeLowReef)),
			elevator->setState(Positions::AlgaeLowReef), grabber->setState(Positions::AlgaeLowReef)).AlongWith(
				setStatePosition(Positions::AlgaeLowReef))).OnlyIf([this] {
			return !intake->isCoralIn();
		});
	}

	frc2::CommandPtr SustainedToAlgaeHighReef() {
		return (frc2::cmd::Parallel(intake->setState(Positions::AlgaeHighReef), arm->setState(Positions::AlgaeHighReef),
			elevator->setState(Positions::AlgaeHighReef), grabber->setState(Positions::AlgaeHighReef)).AlongWith(
				setStatePosition(Positions::AlgaeHighReef))).OnlyIf([this] {
			return !intake->isCoralIn();
		});
	}

	frc2::CommandPtr SustainedToAlgaeGround() {
		return (frc2::cmd::Sequence(intake->setState(Positions::AlgaeGround), arm->setState(Positions::AlgaeGround),
			elevator->setState(Positions::AlgaeGround), grabber->setState(Positions::AlgaeGround)).AlongWith(
				setStatePosition(Positions::AlgaeGround))).OnlyIf([this] {
			return !intake->isCoralIn();
		});
	}

	frc2::CommandPtr SustainedToEndPosition() {
		return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
			elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
			climber->setClimberCommand(ClimberConstants::ClimberOpen)).AlongWith(
				setStatePosition(Positions::EndPosition));
	}

	frc2::CommandPtr IntakeToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !intake->isCoralIn();
		});
	}

	frc2::CommandPtr IntakeToL1Position() {
		return frc2::cmd::Parallel(
			frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
			frc2::cmd::Sequence(intake->setStateRollers(Positions::Through),
				intake->setStateIntake(Positions::L1Position), intake->setState(Positions::L1Position))).AlongWith(
					setStatePosition(Positions::L1Position));
	}

	frc2::CommandPtr AlgaeHighReefToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr AlgaeHighReefToAlgaeHold() {
		return (frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
			intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold)).AlongWith(
				setStatePosition(Positions::AlgaeHold))).OnlyIf([this] {
			return grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr AlgaeLowReefToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr AlgaeLowReefToAlgaeHold() {
		return (frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
			intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold)).AlongWith(
				setStatePosition(Positions::AlgaeHold))).OnlyIf([this] {
			return grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr AlgaeGroundToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr AlgaeGroundToAlgaeHold() {
		return (frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
			intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold)).AlongWith(
				setStatePosition(Positions::AlgaeHold))).OnlyIf([this] {
			return grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr L1PositionToL1Confirm() {
		return frc2::cmd::Parallel(elevator->setState(Positions::L1Confirm), arm->setState(Positions::L1Confirm),
			intake->setState(Positions::L1Confirm), grabber->setState(Positions::L1Confirm)).AlongWith(
				setStatePosition(Positions::L1Confirm));
	}

	frc2::CommandPtr L1PositionToCoralHold() {
		return (frc2::cmd::Sequence(arm->setState(Positions::CoralHold), intake->setStateRollers(Positions::Through),
			intake->setStateIntake(Positions::CoralHold), grabber->setState(Positions::CoralHold),

			elevator->setState(Positions::CoralHold), intake->setState(Positions::CoralHold)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn() && intake->isCoralIn() && (state == Positions::L1Position || state == Positions::Intake);
		});
	}

	frc2::CommandPtr L1PositionToCoralHoldAuto() {
		return (frc2::cmd::Sequence(arm->setState(Positions::CoralHold), intake->setStateRollers(Positions::Through),
			intake->setStateIntake(Positions::CoralHold), grabber->setState(Positions::CoralHold),

			elevator->setState(Positions::CoralHold), intake->setState(Positions::CoralHold)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn();
		});
	}

	frc2::CommandPtr L1ConfirmToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !intake->isCoralIn();
		});
	}

	frc2::CommandPtr CoralHoldToL1Position() {
		return (frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::CoralHold),
			intake->setState(Positions::SustainToL1), grabber->setState(Positions::CoralSpit)).AlongWith(
				setStatePosition(Positions::L1Position))).OnlyIf([this] {
			return (state == Positions::SustainedPosition && intake->isCoralIn());
		});
	}

	frc2::CommandPtr CoralHoldToL2Front() {
		return frc2::cmd::Parallel(
			frc2::cmd::Sequence(arm->setState(Positions::L2Front, Heading::Front),
				elevator->setState(Positions::L2Front)), intake->setState(Positions::L2Front),
			grabber->setState(Positions::L2Front)).AlongWith(setStatePosition(Positions::L2Front));
	}

	frc2::CommandPtr CoralHoldToL3Front() {
		return frc2::cmd::Parallel(arm->setState(Positions::L3Front, Heading::Front),
			elevator->setState(Positions::L3Front), intake->setState(Positions::L3Front),
			grabber->setState(Positions::L3Front)).AlongWith(setStatePosition(Positions::L3Front));
	}

	frc2::CommandPtr CoralHoldToL4Front() {
		return frc2::cmd::Sequence(arm->setState(Positions::L4Front, Heading::Front),
			elevator->setState(Positions::L4Front), intake->setState(Positions::L4Front),
			grabber->setState(Positions::L4Front)).AlongWith(setStatePosition(Positions::L4Front));
	}

	frc2::CommandPtr CoralHoldToL2Back() {
		return frc2::cmd::Parallel(
			frc2::cmd::Sequence(arm->setState(Positions::L2Back, Heading::Back),
				elevator->setState(Positions::L2Back)), intake->setState(Positions::L2Back),
			grabber->setState(Positions::L2Back)).AlongWith(setStatePosition(Positions::L2Back));
	}

	frc2::CommandPtr CoralHoldToL3Back() {
		return frc2::cmd::Parallel(arm->setState(Positions::L3Back, Heading::Back),
			elevator->setState(Positions::L3Back), intake->setState(Positions::L3Back),
			grabber->setState(Positions::L3Back)).AlongWith(setStatePosition(Positions::L3Back));
	}

	frc2::CommandPtr CoralHoldToL4Back() {
		return frc2::cmd::Parallel(arm->setState(Positions::L4Back, Heading::Back),
			elevator->setState(Positions::L4Back), intake->setState(Positions::L4Back),
			grabber->setState(Positions::L4Back)).AlongWith(setStatePosition(Positions::L4Back));
	}

	frc2::CommandPtr AlgaeHoldToNet() {
		return (frc2::cmd::Sequence(arm->setState(Positions::NetPosition), elevator->setState(Positions::NetPosition),
			grabber->setState(Positions::NetPosition), intake->setState(Positions::NetPosition)).AlongWith(
				setStatePosition(Positions::NetPosition))).OnlyIf([this] {
			return grabber->isAlgaeIn();
		});
	}

	frc2::CommandPtr AlgaeHoldToProcessor() {
		return (frc2::cmd::Sequence(intake->setState(Positions::ProcessorPosition),
			arm->setState(Positions::ProcessorPosition), elevator->setState(Positions::ProcessorPosition),
			grabber->setState(Positions::ProcessorPosition)).AlongWith(
				setStatePosition(Positions::ProcessorPosition))).OnlyIf([this] {
			return grabber->isAlgaeIn();
		});
	}

	frc2::CommandPtr AlgaeHoldToEndPosition() {
		return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
			intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
			climber->setClimberCommand(ClimberConstants::ClimberOpen)).AlongWith(
				setStatePosition(Positions::EndPosition));
	}

	frc2::CommandPtr L2FrontToFrontConfirm() {
		return frc2::cmd::Parallel(elevator->setState(Positions::L2FrontConfirm),
			intake->setState(Positions::L2FrontConfirm), arm->setState(Positions::L2FrontConfirm, Heading::Front),
			grabber->setState(Positions::L2FrontConfirm)).AlongWith(
				setStatePosition(Positions::L2FrontConfirm));
	}

	frc2::CommandPtr L3FrontToFrontConfirm() {
		return frc2::cmd::Parallel(elevator->setState(Positions::L3FrontConfirm),
			intake->setState(Positions::L3FrontConfirm), arm->setState(Positions::L3FrontConfirm, Heading::Front),
			grabber->setState(Positions::L3FrontConfirm)).AlongWith(
				setStatePosition(Positions::L3FrontConfirm));
	}

	frc2::CommandPtr L4FrontToFrontConfirm() {
		return frc2::cmd::Parallel(elevator->setState(Positions::L4FrontConfirm),
			intake->setState(Positions::L4FrontConfirm), arm->setState(Positions::L4FrontConfirm, Heading::Front),
			grabber->setState(Positions::L4FrontConfirm)).AlongWith(
				setStatePosition(Positions::L4FrontConfirm));
	}

	frc2::CommandPtr L4FrontAutoToFrontAutoConfirm() {
		return frc2::cmd::Parallel(arm->setState(Positions::L4FrontAutoConfirm, Heading::Front),
			elevator->setState(Positions::L4FrontConfirm), intake->setState(Positions::L4FrontConfirm),
			grabber->setState(Positions::L4FrontConfirm)).AlongWith(
				setStatePosition(Positions::L4FrontAutoConfirm));
	}

	frc2::CommandPtr L2BackToBackConfirm() {
		return frc2::cmd::Parallel(arm->setState(Positions::L2BackConfirm, Heading::Back),
			elevator->setState(Positions::L2BackConfirm), intake->setState(Positions::L2BackConfirm),
			grabber->setState(Positions::L2BackConfirm)).AlongWith(setStatePosition(Positions::L2BackConfirm));
	}

	frc2::CommandPtr L3BackToBackConfirm() {
		return frc2::cmd::Parallel(arm->setState(Positions::L3BackConfirm, Heading::Back),
			elevator->setState(Positions::L3BackConfirm), intake->setState(Positions::L3BackConfirm),
			grabber->setState(Positions::L3BackConfirm)).AlongWith(setStatePosition(Positions::L3BackConfirm));
	}

	frc2::CommandPtr L4BackToBackConfirm() {
		return frc2::cmd::Parallel(arm->setState(Positions::L4BackConfirm, Heading::Back),
			elevator->setState(Positions::L4BackConfirm), intake->setState(Positions::L4BackConfirm),
			grabber->setState(Positions::L4BackConfirm)).AlongWith(setStatePosition(Positions::L4BackConfirm));
	}

	frc2::CommandPtr NetPositionToNetConfirm() {
		return (frc2::cmd::Sequence(grabber->setState(Positions::AlgaeTension), grabber->setState(Positions::NetConfirm),
			arm->setState(Positions::NetConfirm), elevator->setState(Positions::NetConfirm),
			intake->setState(Positions::NetConfirm)).AlongWith(setStatePosition(Positions::NetConfirm))).OnlyIf(
				[this] {
			return grabber->isAlgaeIn();
		});
	}

	frc2::CommandPtr ProcessorPositionToProcessorConfirm() {
		return (frc2::cmd::Sequence(grabber->setState(Positions::ProcessorConfirm),
			intake->setState(Positions::ProcessorConfirm), arm->setState(Positions::ProcessorConfirm),
			elevator->setState(Positions::ProcessorConfirm)).AlongWith(
				setStatePosition(Positions::ProcessorConfirm))).OnlyIf([this] {
			return grabber->isAlgaeIn();
		});
	}

	frc2::CommandPtr FrontConfirmToSustained() {
		return frc2::cmd::Sequence(grabber->setState(Positions::SustainedPosition), elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition));
	}

	frc2::CommandPtr BackConfirmToSustained() {
		return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition));
	}

	frc2::CommandPtr NetConfirmToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr ProcessorConfirmToSustained() {
		return (frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
			arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
			grabber->setState(Positions::SustainedPosition)).AlongWith(
				setStatePosition(Positions::SustainedPosition))).OnlyIf([this] {
			return !grabber->isAlgaeIn() && !intake->isCoralIn();
		});
	}

	frc2::CommandPtr AllToInitial() {
		return frc2::cmd::Sequence(intake->setState(Positions::InitialPosition),
			arm->setState(Positions::InitialPosition), elevator->setState(Positions::InitialPosition),
			grabber->setState(Positions::InitialPosition));
	}

private:
	// bool execute = false;
	Positions desiredState = Positions::InitialPosition;

	Intake* intake = nullptr;
	Arm* arm = nullptr;
	Elevator* elevator = nullptr;
	Grabber* grabber = nullptr;
	Climber* climber = nullptr;
	AlignManager* alignManager = nullptr;

};
