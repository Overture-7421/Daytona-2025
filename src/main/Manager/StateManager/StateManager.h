// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Manager/Transitions/Transitions.h"
#include "Manager/AlignManager/AlignManager.h"
#include "Subsystems/Elevator/Elevator.h"
#include "Subsystems/Arm/Arm.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/Grabber/Grabber.h"
#include "Subsystems/Climber/Climber.h"
#include <vector>

class StateManager {
public:
	StateManager();

	Positions getState();
	frc2::CommandPtr setState(Positions state);

private:

	Intake* intake;
	Arm* arm;
	Elevator* elevator;
	Grabber* grabber;
	Climber* climber;

	AlignManager* alignManager;

	Positions state = Positions::InitialPosition;
	std::vector<Transitions> transitionsMap = {

	  {Positions::InitialPosition, Positions::SustainedPosition, [this]() {return frc::DriverStation::IsEnabled();}},
	  {Positions::InitialPosition, Positions::L2Front, [this]() {return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();}},
	  {Positions::InitialPosition, Positions::L3Front, [this]() {return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();}},
	  {Positions::InitialPosition, Positions::L4Front, [this]() {return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();}},
	  {Positions::InitialPosition, Positions::L2Back, [this]() {return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();}},
	  {Positions::InitialPosition, Positions::L3Back, [this]() {return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();}},
	  {Positions::InitialPosition, Positions::L4Back, [this]() {return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();}},

	  {Positions::SustainedPosition, Positions::Intake, [this]() {return !grabber->isCoralIn() && !intake->isCoralIn();}},
	  {Positions::SustainedPosition, Positions::IntakeCoralStation, [this]() {return !grabber->isCoralIn() && !intake->isCoralIn();}},
	  {Positions::SustainedPosition, Positions::AlgaeLowReef, [this]() {return !grabber->isCoralIn() && !intake->isCoralIn() }},
	  {Positions::SustainedPosition, Positions::AlgaeHighReef, [this]() {return !grabber->isCoralIn() && !intake->isCoralIn() ;}},
	  {Positions::SustainedPosition, Positions::AlgaeGround, [this]() {return !grabber->isCoralIn() && !intake->isCoralIn() ;}},
	  {Positions::SustainedPosition, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::Intake, Positions::SustainedPosition, [this]() {return false;/*Boton Respectivo*/}},
	  {Positions::Intake, Positions::L1Position, [this]() {return intake->isCoralIn();}},
	  {Positions::Intake, Positions::CoralAndAlgae, [this]() {return intake->isCoralIn() && grabber->isAlgaeIn();}},
	  {Positions::Intake, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::IntakeCoralStation, Positions::SustainedPosition, [this]() {return false; /*Boton Respectivo*/}},
	  {Positions::IntakeCoralStation, Positions::L1Position, [this]() {return intake->isCoralIn();}},
	  {Positions::IntakeCoralStation, Positions::CoralAndAlgae, [this]() {return intake->isCoralIn() && grabber->isAlgaeIn();}},
	  {Positions::IntakeCoralStation, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::AlgaeHighReef, Positions::SustainedPosition, [this]() {return false; /*Boton Respectivo*/}},
	  {Positions::AlgaeHighReef, Positions::CoralAndAlgae, [this]() {return intake->isCoralIn() && grabber->isAlgaeIn();}},
	  {Positions::AlgaeHighReef, Positions::AlgaeHold, [this]() {return grabber->isAlgaeIn();}},
	  {Positions::AlgaeHighReef, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::AlgaeLowReef, Positions::SustainedPosition, [this]() {return false; /*Boton Respectivo*/}},
	  {Positions::AlgaeLowReef, Positions::CoralAndAlgae, [this]() {return intake->isCoralIn() && grabber->isAlgaeIn();}},
	  {Positions::AlgaeLowReef, Positions::AlgaeHold, [this]() {return grabber->isAlgaeIn();}},
	  {Positions::AlgaeLowReef, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::AlgaeGround, Positions::SustainedPosition, [this]() {return false; /*Boton Respectivo*/}},
	  {Positions::AlgaeGround, Positions::CoralAndAlgae, [this]() {return intake->isCoralIn() && grabber->isAlgaeIn();}},
	  {Positions::AlgaeGround, Positions::AlgaeHold, [this]() {return grabber->isAlgaeIn();}},
	  {Positions::AlgaeGround, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::L1Position, Positions::L1Confirm, [this]() {return false; /*Boton Respectivo*/}},
	  {Positions::L1Position, Positions::CoralHold, [this]() {return false /*Boton Respectivo*/ && !grabber->isAlgaeIn() && intake->isCoralIn();}},
	  {Positions::L1Position, Positions::CoralAndAlgae, [this]() {return !grabber->isCoralIn() & !grabber->isAlgaeIn() && false /*Boton Respectivo*/ ;}},
	  {Positions::L1Position, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::L1Confirm, Positions::SustainedPosition, [this]() {return !grabber->isCoralIn();},

	  {Positions::CoralHold, Positions::L1Position, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/;}},
	  {Positions::CoralHold, Positions::L2Front, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/ && /*chassis*/ ;}},
	  {Positions::CoralHold, Positions::L3Front, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/ && /*chassis*/ ;}},
	  {Positions::CoralHold, Positions::L4Front, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/ && /*chassis*/ ;}},
	  {Positions::CoralHold, Positions::L2Back, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/ && /*chassis*/ ;}},
	  {Positions::CoralHold, Positions::L3Back, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/ && /*chassis*/ ;}},
	  {Positions::CoralHold, Positions::L4Back, [this]() {return grabber->isCoralIn() && false /*Boton Respectivo*/ && /*chassis*/ ;}},
	  {Positions::CoralHold, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::CoralAndAlgae, Positions::CoralHold, [this]() {return !grabber->isAlgaeIn()}},
	  {Positions::CoralAndAlgae, Positions::NetPosition, [this]() {return grabber->isAlgaeIn() && return false; /*Boton Respectivo*/}},
	  {Positions::CoralAndAlgae, Positions::ProcessorPosition, [this]() {return grabber->isAlgaeIn() && return false; /*Boton Respectivo*/}},
	  {Positions::CoralAndAlgae, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::AlgaeHold, Positions::NetPosition, [this]() {return grabber->isAlgaeIn() && return false; /*Boton Respectivo*/}},
	  {Positions::AlgaeHold, Positions::ProcessorPosition, [this]() {return grabber->isAlgaeIn() && return false; /*Boton Respectivo*/}},
	  {Positions::AlgaeHold, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::L2Front, Positions::L2FrontConfirm [this]() {return grabber->isCoralIn() && return false; /*suma de dos botones*/}},
	  {Positions::L3Front, Positions::L3FrontConfirm [this]() {return grabber->isCoralIn() && return false; /*suma de dos botones*/}},
	  {Positions::L4Front, Positions::L4FrontConfirm [this]() {return grabber->isCoralIn() && return false; /*suma de dos botones*/}},

	  {Positions::L2Back, Positions::L2BackConfirm [this]() {return grabber->isCoralIn() && return false; /*suma de dos botones*/}},
	  {Positions::L3Back, Positions::L3BackConfirm [this]() {return grabber->isCoralIn() && return false; /*suma de dos botones*/}},
	  {Positions::L4Back, Positions::L4BackConfirm [this]() {return grabber->isCoralIn() && return false; /*suma de dos botones*/}},

	  {Positions::NetPosition, Positions::NetConfirm [this]() {return grabber->isAlgaeIn() && return false; /*Boton Respectivo*/}},
	  {Positions::NetPosition, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::ProcessorPosition, Positions::ProcessorConfirm [this]() {return grabber->isAlgaeIn() && return false; /*Boton Respectivo*/}},\
	  {Positions::ProcessorPosition, Positions::EndPosition, [this]() {return false /*boton especifico*/;}},

	  {Positions::L2FrontConfirm, Positions::SustainedPosition [this]() {return !grabber->isCoralIn();}},
	  {Positions::L3FrontConfirm, Positions::SustainedPosition [this]() {return !grabber->isCoralIn();}},
	  {Positions::L4FrontConfirm, Positions::SustainedPosition [this]() {return !grabber->isCoralIn();}},

	  {Positions::L2BackConfirm, Positions::SustainedPosition [this]() {return !grabber->isCoralIn();}},
	  {Positions::L3BackConfirm, Positions::SustainedPosition [this]() {return !grabber->isCoralIn();}},
	  {Positions::L4BackConfirm, Positions::SustainedPosition [this]() {return !grabber->isCoralIn();}},

	  {Positions::NetConfirm, Positions::SustainedPosition [this]() {return !grabber->isAlgaeIn();}},
	  {Positions::NetConfirm, Positions::CoralHold [this]() {return !grabber->isAlgaeIn() && intake->isCoralIn();}},

	  {Positions::ProcessorConfirm, Positions::SustainedPosition [this]() {return !grabber->isAlgaeIn();}},
	  {Positions::ProcessorConfirm, Positions::CoralHold [this]() {return !grabber->isAlgaeIn() && intake->isCoralIn();}},

	  {Positions::EndPosition, Positions::SustainedPosition [this]() {return false /*Boton especifico*/;}},
	  {Positions::EndPosition, Positions::CoralHold [this]() {return false /*Boton especifico*/ && grabber->isCoralIn();}}

	};

	std::map<Positions, frc2::CommandPtr> positionsMap;


};
