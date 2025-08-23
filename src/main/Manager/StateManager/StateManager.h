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

    std::map<Positions, const frc2::CommandPtr&> positionsMap { 
      {Positions::L1Position, frc2::cmd::Parallel(
        frc2::cmd::Sequence(
        arm->setState(Positions::L1Position),
        elevator->setState(Positions::L1Position)),
        frc2::cmd::Parallel(
        intake->setState(Positions::L1Position),
        grabber->setState(Positions::L1Position),
        climber->setState(Positions::L1Position))
      )},

      {Positions::L1Confirm, frc2::cmd::Parallel(
        elevator->setState(Positions::L1Confirm),
        arm->setState(Positions::L1Confirm),
        intake->setState(Positions::L1Confirm),
        grabber->setState(Positions::L1Confirm),
        climber->setState(Positions::L1Confirm)
      )},

      {Positions::L2Front, frc2::cmd::Parallel(
        frc2::cmd::Sequence(
        arm->setState(Positions::L2Front),
        elevator->setState(Positions::L2Front,Heading::Front)),
        frc2::cmd::Parallel(
        intake->setState(Positions::L2Front),
        grabber->setState(Positions::L2Front),
        climber->setState(Positions::L2Front))
        )
      },

      {Positions::L2FrontConfirm, frc2::cmd::Parallel(
        arm->setState(Positions::L2FrontConfirm),
        elevator->setState(Positions::L2FrontConfirm,Heading::Front)),
        intake->setState(Positions::L2FrontConfirm),
        grabber->setState(Positions::L2FrontConfirm),
        climber->setState(Positions::L2FrontConfirm)
      },

      {Positions::L2Back, frc2::cmd::Parallel(
        frc2::cmd::Sequence(
        arm->setState(Positions::L2Back),
        elevator->setState(Positions::L2Back,Heading::Back)),
        frc2::cmd::Parallel(
        intake->setState(Positions::L2Back),
        grabber->setState(Positions::L2Back),
        climber->setState(Positions::L2Back))
        )
      },

      {Positions::L2BackConfirm, frc2::cmd::Parallel(
        arm->setState(Positions::L2BackConfirm),
        elevator->setState(Positions::L2BackConfirm,Heading::Back)),
        intake->setState(Positions::L2BackConfirm),
        grabber->setState(Positions::L2BackConfirm),
        climber->setState(Positions::L2BackConfirm)
      },

      {Positions::L3Front, frc2::cmd::Parallel(
        arm->setState(Positions::L3Front),
        elevator->setState(Positions::L3Front,Heading::Front)),
        intake->setState(Positions::L3Front),
        grabber->setState(Positions::L3Front),
        climber->setState(Positions::L3Front)
      },

      {Positions::L3FrontConfirm, frc2::cmd::Parallel(
        arm->setState(Positions::L3FrontConfirm),
        elevator->setState(Positions::L3FrontConfirm,Heading::Front)),
        intake->setState(Positions::L3FrontConfirm),
        grabber->setState(Positions::L3FrontConfirm),
        climber->setState(Positions::L3FrontConfirm)
      },

      {Positions::L3Back, frc2::cmd::Parallel(
        arm->setState(Positions::L3Back),
        elevator->setState(Positions::L3Back,Heading::Back)),
        intake->setState(Positions::L3Back),
        grabber->setState(Positions::L3Back),
        climber->setState(Positions::L3Back)
      },

      {Positions::L3BackConfirm, frc2::cmd::Parallel(
        arm->setState(Positions::L3BackConfirm),
        elevator->setState(Positions::L3BackConfirm,Heading::Back)),
        intake->setState(Positions::L3BackConfirm),
        grabber->setState(Positions::L3BackConfirm),
        climber->setState(Positions::L3BackConfirm)
      },

    {Positions::L4Front, frc2::cmd::Parallel(
        arm->setState(Positions::L4Front),
        elevator->setState(Positions::L4Front,Heading::Front)),
        intake->setState(Positions::L4Front),
        grabber->setState(Positions::L4Front),
        climber->setState(Positions::L4Front)
      },

      {Positions::L4FrontConfirm, frc2::cmd::Parallel(
        arm->setState(Positions::L4FrontConfirm),
        elevator->setState(Positions::L4FrontConfirm,Heading::Front)),
        intake->setState(Positions::L4FrontConfirm),
        grabber->setState(Positions::L4FrontConfirm),
        climber->setState(Positions::L4FrontConfirm)
      },

      {Positions::L4Back, frc2::cmd::Parallel(
        arm->setState(Positions::L4Back),
        elevator->setState(Positions::L4Back,Heading::Back)),
        intake->setState(Positions::L4Back),
        grabber->setState(Positions::L4Back),
        climber->setState(Positions::L4Back)
      },

      {Positions::L4BackConfirm, frc2::cmd::Parallel(
        arm->setState(Positions::L4BackConfirm),
        elevator->setState(Positions::L4BackConfirm,Heading::Back)),
        intake->setState(Positions::L4BackConfirm),
        grabber->setState(Positions::L4BackConfirm),
        climber->setState(Positions::L4BackConfirm)
      },
    };

};
