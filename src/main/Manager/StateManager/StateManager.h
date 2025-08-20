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
    std::vector<Transitions> transitionsMap = { 
      {Positions::InitialPosition, Positions::SustainedPosition, [this]() {return frc::DriverStation::IsEnabled();}}, 
      {Positions::InitialPosition, Positions::L2Front, [this]() {
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

    std::map<Positions, frc2::CommandPtr> positionsMap;


};
