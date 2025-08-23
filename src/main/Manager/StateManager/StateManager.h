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

    std::map<Positions, const frc2::CommandPtr&> positionsMap { {Positions::InitialPosition, frc2::cmd::Parallel(
            elevator->setState(Positions::InitialPosition), arm->setState(Positions::InitialPosition),
            intake->setState(Positions::InitialPosition), grabber->setState(Positions::InitialPosition),
            climber->setState(Positions::InitialPosition))}, {Positions::SustainedPosition, frc2::cmd::Sequence(
            elevator->setState(Positions::SustainedPosition), arm->setState(Positions::SustainedPosition),
            intake->setState(Positions::SustainedPosition), grabber->setState(Positions::SustainedPosition),
            climber->setState(Positions::SustainedPosition))}, {Positions::Intake, frc2::cmd::Sequence(
            intake->setState(Positions::Intake), elevator->setState(Positions::Intake),
            arm->setState(Positions::Intake), grabber->setState(Positions::Intake),
            climber->setState(Positions::Intake))}, {Positions::IntakeCoralStation, frc2::cmd::Sequence(
            intake->setState(Positions::IntakeCoralStation), elevator->setState(Positions::IntakeCoralStation),
            arm->setState(Positions::IntakeCoralStation), grabber->setState(Positions::IntakeCoralStation),
            climber->setState(Positions::IntakeCoralStation))}, {Positions::AlgaeHighReef, frc2::cmd::Sequence(
            elevator->setState(Positions::AlgaeHighReef), arm->setState(Positions::AlgaeHighReef),
            grabber->setState(Positions::AlgaeHighReef), intake->setState(Positions::AlgaeHighReef),
            climber->setState(Positions::AlgaeHighReef))}, {Positions::AlgaeLowReef, frc2::cmd::Sequence(
            elevator->setState(Positions::AlgaeLowReef), arm->setState(Positions::AlgaeLowReef),
            grabber->setState(Positions::AlgaeLowReef), intake->setState(Positions::AlgaeLowReef),
            climber->setState(Positions::AlgaeLowReef))}, {Positions::AlgaeGround, frc2::cmd::Sequence(
            intake->setState(Positions::AlgaeGround), arm->setState(Positions::AlgaeGround),
            elevator->setState(Positions::AlgaeGround), grabber->setState(Positions::AlgaeGround),
            climber->setState(Positions::AlgaeGround))}, {Positions::CoralHold, frc2::cmd::Sequence(
            intake->setState(Positions::CoralHold), arm->setState(Positions::CoralHold),
            elevator->setState(Positions::CoralHold), grabber->setState(Positions::CoralHold),
            climber->setState(Positions::CoralHold))}, {Positions::CoralAndAlgae, frc2::cmd::Sequence(
            elevator->setState(Positions::CoralAndAlgae), arm->setState(Positions::CoralAndAlgae),
            intake->setState(Positions::CoralAndAlgae), grabber->setState(Positions::CoralAndAlgae),
            climber->setState(Positions::CoralAndAlgae))}, {Positions::AlgaeHold, frc2::cmd::Sequence(
            arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
            intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold),
            climber->setState(Positions::AlgaeHold))}, {Positions::NetPosition, frc2::cmd::Sequence(
            arm->setState(Positions::NetPosition), elevator->setState(Positions::NetPosition),
            grabber->setState(Positions::NetPosition), intake->setState(Positions::NetPosition),
            climber->setState(Positions::NetPosition))}, {Positions::NetConfirm, frc2::cmd::Sequence(
            grabber->setState(Positions::NetConfirm), arm->setState(Positions::NetConfirm),
            elevator->setState(Positions::NetConfirm), intake->setState(Positions::NetConfirm),
            climber->setState(Positions::NetConfirm))}, {Positions::ProcessorPosition, frc2::cmd::Sequence(
            intake->setState(Positions::ProcessorPosition), arm->setState(Positions::ProcessorPosition),
            elevator->setState(Positions::ProcessorPosition), grabber->setState(Positions::ProcessorPosition),
            climber->setState(Positions::ProcessorPosition))}, {Positions::ProcessorConfirm, frc2::cmd::Sequence(
            grabber->setState(Positions::ProcessorConfirm), intake->setState(Positions::ProcessorConfirm),
            arm->setState(Positions::ProcessorConfirm), elevator->setState(Positions::ProcessorConfirm),
            climber->setState(Positions::ProcessorConfirm))}, {Positions::EndPosition, frc2::cmd::Sequence(
            arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
            intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
            climber->setState(Positions::EndPosition))}, {Positions::L1Position, frc2::cmd::Parallel(
            frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
            frc2::cmd::Parallel(intake->setState(Positions::L1Position), grabber->setState(Positions::L1Position),
                    climber->setState(Positions::L1Position)))},

    {Positions::L1Confirm, frc2::cmd::Parallel(elevator->setState(Positions::L1Confirm),
            arm->setState(Positions::L1Confirm), intake->setState(Positions::L1Confirm),
            grabber->setState(Positions::L1Confirm), climber->setState(Positions::L1Confirm))},

    {Positions::L2Front, frc2::cmd::Parallel(
            frc2::cmd::Sequence(arm->setState(Positions::L2Front, Heading::Front),
                    elevator->setState(Positions::L2Front)),
            frc2::cmd::Parallel(intake->setState(Positions::L2Front), grabber->setState(Positions::L2Front),
                    climber->setState(Positions::L2Front)))},

    {Positions::L2FrontConfirm, frc2::cmd::Parallel(arm->setState(Positions::L2FrontConfirm, Heading::Front),
            elevator->setState(Positions::L2FrontConfirm), intake->setState(Positions::L2FrontConfirm),
            grabber->setState(Positions::L2FrontConfirm), climber->setState(Positions::L2FrontConfirm))},

    {Positions::L2Back, frc2::cmd::Parallel(
            frc2::cmd::Sequence(arm->setState(Positions::L2Back, Heading::Back), elevator->setState(Positions::L2Back)),
            frc2::cmd::Parallel(intake->setState(Positions::L2Back), grabber->setState(Positions::L2Back),
                    climber->setState(Positions::L2Back)))},

    {Positions::L2BackConfirm, frc2::cmd::Parallel(arm->setState(Positions::L2BackConfirm, Heading::Back),
            elevator->setState(Positions::L2BackConfirm), intake->setState(Positions::L2BackConfirm),
            grabber->setState(Positions::L2BackConfirm), climber->setState(Positions::L2BackConfirm))},

    {Positions::L3Front, frc2::cmd::Parallel(arm->setState(Positions::L3Front, Heading::Front),
            elevator->setState(Positions::L3Front), intake->setState(Positions::L3Front),
            grabber->setState(Positions::L3Front), climber->setState(Positions::L3Front))},

    {Positions::L3FrontConfirm, frc2::cmd::Parallel(arm->setState(Positions::L3FrontConfirm, Heading::Front),
            elevator->setState(Positions::L3FrontConfirm), intake->setState(Positions::L3FrontConfirm),
            grabber->setState(Positions::L3FrontConfirm), climber->setState(Positions::L3FrontConfirm))},

    {Positions::L3Back, frc2::cmd::Parallel(arm->setState(Positions::L3Back, Heading::Back),
            elevator->setState(Positions::L3Back), intake->setState(Positions::L3Back),
            grabber->setState(Positions::L3Back), climber->setState(Positions::L3Back))},

    {Positions::L3BackConfirm, frc2::cmd::Parallel(arm->setState(Positions::L3BackConfirm, Heading::Back),
            elevator->setState(Positions::L3BackConfirm), intake->setState(Positions::L3BackConfirm),
            grabber->setState(Positions::L3BackConfirm), climber->setState(Positions::L3BackConfirm))},

    {Positions::L4Front, frc2::cmd::Parallel(

    arm->setState(Positions::L4Front, Heading::Front), elevator->setState(Positions::L4Front)

    , intake->setState(Positions::L4Front), grabber->setState(Positions::L4Front),
            climber->setState(Positions::L4Front))},

    {Positions::L4FrontConfirm, frc2::cmd::Parallel(arm->setState(Positions::L4FrontConfirm, Heading::Front),
            elevator->setState(Positions::L4FrontConfirm), intake->setState(Positions::L4FrontConfirm),
            grabber->setState(Positions::L4FrontConfirm), climber->setState(Positions::L4FrontConfirm))},

    {Positions::L4Back, frc2::cmd::Parallel(arm->setState(Positions::L4Back, Heading::Back),
            elevator->setState(Positions::L4Back), intake->setState(Positions::L4Back),
            grabber->setState(Positions::L4Back), climber->setState(Positions::L4Back))},

    {Positions::L4BackConfirm, frc2::cmd::Parallel(arm->setState(Positions::L4BackConfirm, Heading::Back),
            elevator->setState(Positions::L4BackConfirm), intake->setState(Positions::L4BackConfirm),
            grabber->setState(Positions::L4BackConfirm), climber->setState(Positions::L4BackConfirm))}

    };

};
