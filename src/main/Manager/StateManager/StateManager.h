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

    Intake *intake;
    Arm *arm;
    Elevator *elevator;
    Grabber *grabber;
    Climber *climber;

    AlignManager *alignManager;

    Positions state = Positions::InitialPosition;
    std::vector<Transitions> transitionsMap = { {Positions::InitialPosition, Positions::SustainedPosition, [this]() {
        return frc::DriverStation::IsEnabled();
    }}, {Positions::InitialPosition, Positions::L2Front, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::InitialPosition, Positions::L3Front, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::InitialPosition, Positions::L4Front, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::InitialPosition, Positions::L2Back, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::InitialPosition, Positions::L3Back, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::InitialPosition, Positions::L4Back, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::InitialPosition, Positions::EndPosition, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }}, {Positions::SustainedPosition, Positions::Intake, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }}, {Positions::SustainedPosition, Positions::IntakeCoralStation, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }}, {Positions::SustainedPosition, Positions::AlgaeLowReef, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }}, {Positions::SustainedPosition, Positions::AlgaeHighReef, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }}, {Positions::SustainedPosition, Positions::AlgaeGround, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }}, {Positions::Intake, Positions::SustainedPosition, [this]() {
        return false;/*Boton Respectivo*/
    }}, {Positions::Intake, Positions::L1Position, [this]() {
        return intake->isCoralIn();
    }}

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
            climber->setState(Positions::EndPosition))}

    };

};
