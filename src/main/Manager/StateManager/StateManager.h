// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "Manager/Transitions/Transitions.h"
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
	StateManager(Intake* intake, Arm* arm, Elevator* elevator, Grabber* grabber, Climber* climber);

	void Periodic() override;

	Positions getStatePosition();

	frc2::CommandPtr setStatePosition(Positions desiredState);
	frc2::CommandPtr setStateOverride();

	Positions state = Positions::InitialPosition;
	
	frc2::CommandPtr InitialToSustained();
	frc2::CommandPtr InitialToL4Front();
	frc2::CommandPtr InitialToL4Back();
	frc2::CommandPtr SustainedToIntake();
	frc2::CommandPtr SustainedToAlgaeLowReef() ;
	frc2::CommandPtr SustainedToAlgaeHighReef();
	frc2::CommandPtr SustainedToAlgaeGround();
	frc2::CommandPtr SustainedToEndPosition();
	frc2::CommandPtr IntakeToL1Position();
	frc2::CommandPtr AlgaeHighReefToSustained();
	frc2::CommandPtr AlgaeHighReefToAlgaeHold();
	frc2::CommandPtr AlgaeLowReefToSustained();
	frc2::CommandPtr AlgaeLowReefToAlgaeHold();
	frc2::CommandPtr AlgaeGroundToSustained();
	frc2::CommandPtr AlgaeGroundToAlgaeHold();
	frc2::CommandPtr L1PositionToL1Confirm();
	frc2::CommandPtr L1PositionToCoralHold();
	frc2::CommandPtr L1PositionToCoralHoldAuto();
	frc2::CommandPtr L1ConfirmToSustained();
	frc2::CommandPtr CoralHoldToL1Position();
	frc2::CommandPtr CoralHoldToL2Front();
	frc2::CommandPtr CoralHoldToL3Front();
	frc2::CommandPtr CoralHoldToL4Front();
	frc2::CommandPtr CoralHoldToL2Back();
	frc2::CommandPtr CoralHoldToL3Back();
	frc2::CommandPtr CoralHoldToL4Back();
	frc2::CommandPtr AlgaeHoldToNet();
	frc2::CommandPtr AlgaeHoldToProcessor();
	frc2::CommandPtr L2FrontToFrontConfirm();
	frc2::CommandPtr L3FrontToFrontConfirm();
	frc2::CommandPtr L4FrontToFrontConfirm();
	frc2::CommandPtr L4FrontAutoToFrontAutoConfirm();
	frc2::CommandPtr L2BackToBackConfirm();
	frc2::CommandPtr L3BackToBackConfirm();
	frc2::CommandPtr L4BackToBackConfirm();
	frc2::CommandPtr NetPositionToNetConfirm();
	frc2::CommandPtr ProcessorPositionToProcessorConfirm();
	frc2::CommandPtr FrontConfirmToSustained();
	frc2::CommandPtr BackConfirmToSustained();
	frc2::CommandPtr NetConfirmToSustained();
	frc2::CommandPtr ProcessorConfirmToSustained();
	frc2::CommandPtr AllToInitial();

private:
	// bool execute = false;
	Positions desiredState = Positions::InitialPosition;

	Intake* intake = nullptr;
	Arm* arm = nullptr;
	Elevator* elevator = nullptr;
	Grabber* grabber = nullptr;
	Climber* climber = nullptr;
};
