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
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>
#include <OvertureLib/Gamepads/OverConsole/OverConsole.h>

class StateManager {
public:
    StateManager(Intake *intake, Arm *arm, Elevator *elevator, Grabber *grabber, Climber *climber,
            OverXboxController *driver, OverXboxController *oprtr, OverConsole *console, frc2::Trigger *endToInitial);

    Positions getStatePosition();
    frc2::CommandPtr setStatePosition(Positions state);
    frc2::CommandPtr setStateOverride();

private:

    Intake *intake;
    Arm *arm;
    Elevator *elevator;
    Grabber *grabber;
    Climber *climber;

    OverXboxController *driver;
    OverXboxController *oprtr;
    OverConsole *console;

    frc2::Trigger *endToInitial;

    AlignManager *alignManager;

    frc2::CommandPtr commandScheduled = frc2::cmd::None();

    Positions state = Positions::InitialPosition;
    //No se define en que estado empieza, ahorita vemos eso

    Transitions *current = nullptr;
    std::vector<Transitions> transitionsMap = {

    {Positions::InitialPosition, Positions::SustainedPosition, [this]() {
        return frc::DriverStation::IsEnabled();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::InitialPosition), elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition),
                climber->servoAngleCommand(ClimberConstants::ClosedServo));
    }}, {Positions::InitialPosition, Positions::L2Front, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Parallel(

                frc2::cmd::Sequence(arm->setState(Positions::L2Front, Heading::Front),
                        elevator->setState(Positions::L2Front), climber->setState(Positions::L2Front),
                        climber->servoAngleCommand(ClimberConstants::ClosedServo)),
                intake->setState(Positions::L2Front), grabber->setState(Positions::L2Front));
    }}, {Positions::InitialPosition, Positions::L3Front, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L3Front, Heading::Front),
                elevator->setState(Positions::L3Front), intake->setState(Positions::L3Front),
                grabber->setState(Positions::L3Front),
                frc2::cmd::Sequence(climber->setState(Positions::L3Front),
                        climber->servoAngleCommand(ClimberConstants::ClosedServo)));
    }}, {Positions::InitialPosition, Positions::L4Front, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L4Front, Heading::Front),
                elevator->setState(Positions::L4Front), intake->setState(Positions::L4Front),
                grabber->setState(Positions::L4Front),
                frc2::cmd::Sequence(climber->setState(Positions::L4Front),
                        climber->servoAngleCommand(ClimberConstants::ClosedServo)));
    }}, {Positions::InitialPosition, Positions::L2Back, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L2Back, Heading::Back),
                        elevator->setState(Positions::L2Back), climber->setState(Positions::L2Back),
                        climber->servoAngleCommand(ClimberConstants::ClosedServo)), intake->setState(Positions::L2Back),
                grabber->setState(Positions::L2Back));
    }}, {Positions::InitialPosition, Positions::L3Back, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L3Back, Heading::Back),
                elevator->setState(Positions::L3Back), intake->setState(Positions::L3Back),
                grabber->setState(Positions::L3Back),
                frc2::cmd::Sequence(climber->setState(Positions::L3Back),
                        climber->servoAngleCommand(ClimberConstants::ClosedServo)));
    }}, {Positions::InitialPosition, Positions::L4Back, [this]() {
        return frc::DriverStation::IsAutonomous() && grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L4Back, Heading::Back),
                elevator->setState(Positions::L4Back), intake->setState(Positions::L4Back),
                grabber->setState(Positions::L4Back),
                frc2::cmd::Sequence(climber->setState(Positions::L4Back),
                        climber->servoAngleCommand(ClimberConstants::ClosedServo)));
    }},

    {Positions::SustainedPosition, Positions::Intake, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::Intake), elevator->setState(Positions::Intake),
                arm->setState(Positions::Intake), grabber->setState(Positions::Intake),
                climber->setState(Positions::Intake));
    }}, {Positions::SustainedPosition, Positions::IntakeCoralStation, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::IntakeCoralStation),
                elevator->setState(Positions::IntakeCoralStation), arm->setState(Positions::IntakeCoralStation),
                grabber->setState(Positions::IntakeCoralStation), climber->setState(Positions::IntakeCoralStation));
    }}, {Positions::SustainedPosition, Positions::AlgaeLowReef, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn() && (alignManager->getAlgaePose() == AlgaePose::Down);
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::AlgaeLowReef), arm->setState(Positions::AlgaeLowReef),
                grabber->setState(Positions::AlgaeLowReef), intake->setState(Positions::AlgaeLowReef),
                climber->setState(Positions::AlgaeLowReef));
    }}, {Positions::SustainedPosition, Positions::AlgaeHighReef, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn() && (alignManager->getAlgaePose() == AlgaePose::Up);
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::AlgaeHighReef),
                arm->setState(Positions::AlgaeHighReef), grabber->setState(Positions::AlgaeHighReef),
                intake->setState(Positions::AlgaeHighReef), climber->setState(Positions::AlgaeHighReef));
    }}, {Positions::SustainedPosition, Positions::AlgaeGround, [this]() {
        return !grabber->isCoralIn() && !intake->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::AlgaeGround), arm->setState(Positions::AlgaeGround),
                elevator->setState(Positions::AlgaeGround), grabber->setState(Positions::AlgaeGround),
                climber->setState(Positions::AlgaeGround));
    }}, {Positions::SustainedPosition, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::Intake, Positions::SustainedPosition, [this]() {
        return !driver->LeftTrigger().Get() && !intake->isCoralIn() && !grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::Intake, Positions::L1Position, [this]() {
        return intake->isCoralIn() && driver->LeftTrigger().Get();
    }, [this]() {
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
                intake->setState(Positions::L1Position), grabber->setState(Positions::L1Position),
                climber->setState(Positions::L1Position));
    }}, {Positions::Intake, Positions::CoralAndAlgae, [this]() {
        return intake->isCoralIn() && grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::CoralAndAlgae),
                arm->setState(Positions::CoralAndAlgae), intake->setState(Positions::CoralAndAlgae),
                grabber->setState(Positions::CoralAndAlgae), climber->setState(Positions::CoralAndAlgae));
    }}, {Positions::Intake, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::IntakeCoralStation, Positions::SustainedPosition, [this]() {
        return (!oprtr->RightBumper().Get() || !console->AxisMagnitudeGreaterThan(0, 0.1).Get()) && !intake->isCoralIn()
                && !grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::IntakeCoralStation, Positions::L1Position, [this]() {
        return intake->isCoralIn() && driver->LeftTrigger().Get();
    }, [this]() {
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
                intake->setState(Positions::L1Position), grabber->setState(Positions::L1Position),
                climber->setState(Positions::L1Position));
    }}, {Positions::IntakeCoralStation, Positions::CoralAndAlgae, [this]() {
        return intake->isCoralIn() && grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::CoralAndAlgae),
                arm->setState(Positions::CoralAndAlgae), intake->setState(Positions::CoralAndAlgae),
                grabber->setState(Positions::CoralAndAlgae), climber->setState(Positions::CoralAndAlgae));
    }}, {Positions::IntakeCoralStation, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::AlgaeHighReef, Positions::SustainedPosition, [this]() {
        return (!oprtr->POVUp().Get() || !console->Button(1).Get()) && !grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::AlgaeHighReef, Positions::CoralAndAlgae, [this]() {
        return intake->isCoralIn() && grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::CoralAndAlgae),
                arm->setState(Positions::CoralAndAlgae), intake->setState(Positions::CoralAndAlgae),
                grabber->setState(Positions::CoralAndAlgae), climber->setState(Positions::CoralAndAlgae));
    }}, {Positions::AlgaeHighReef, Positions::AlgaeHold, [this]() {
        return grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
                intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold),
                climber->setState(Positions::AlgaeHold));
    }}, {Positions::AlgaeHighReef, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::AlgaeLowReef, Positions::SustainedPosition, [this]() {
        return (!oprtr->POVDown().Get() || !console->Button(1).Get()) && !grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::AlgaeLowReef, Positions::CoralAndAlgae, [this]() {
        return intake->isCoralIn() && grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::CoralAndAlgae),
                arm->setState(Positions::CoralAndAlgae), intake->setState(Positions::CoralAndAlgae),
                grabber->setState(Positions::CoralAndAlgae), climber->setState(Positions::CoralAndAlgae));
    }}, {Positions::AlgaeLowReef, Positions::AlgaeHold, [this]() {
        return grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
                intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold),
                climber->setState(Positions::AlgaeHold));
    }}, {Positions::AlgaeLowReef, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::AlgaeGround, Positions::SustainedPosition, [this]() {
        return !driver->POVLeft().Get() && !grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::AlgaeGround, Positions::CoralAndAlgae, [this]() {
        return intake->isCoralIn() && grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::CoralAndAlgae),
                arm->setState(Positions::CoralAndAlgae), intake->setState(Positions::CoralAndAlgae),
                grabber->setState(Positions::CoralAndAlgae), climber->setState(Positions::CoralAndAlgae));
    }}, {Positions::AlgaeGround, Positions::AlgaeHold, [this]() {
        return grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::AlgaeHold), elevator->setState(Positions::AlgaeHold),
                intake->setState(Positions::AlgaeHold), grabber->setState(Positions::AlgaeHold),
                climber->setState(Positions::AlgaeHold));
    }}, {Positions::AlgaeGround, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::L1Position, Positions::L1Confirm, [this]() {
        return driver->RightBumper().Get() && driver->LeftTrigger().Get();
    }, [this]() {
        return frc2::cmd::Parallel(elevator->setState(Positions::L1Confirm), arm->setState(Positions::L1Confirm),
                intake->setState(Positions::L1Confirm), grabber->setState(Positions::L1Confirm),
                climber->setState(Positions::L1Confirm));
    }}, {Positions::L1Position, Positions::CoralHold, [this]() {
        return !driver->LeftTrigger().Get() && !grabber->isAlgaeIn() && intake->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::CoralHold), arm->setState(Positions::CoralHold),
                elevator->setState(Positions::CoralHold), grabber->setState(Positions::CoralHold),
                climber->setState(Positions::CoralHold));
    }}, {Positions::L1Position, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::EndPosition), arm->setState(Positions::EndPosition),
                elevator->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::L1Confirm, Positions::SustainedPosition, [this]() {
        return !grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }},

    {Positions::CoralHold, Positions::L1Position, [this]() {
        return grabber->isCoralIn() && driver->POVUp().Get();
    }, [this]() {
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L1Position), elevator->setState(Positions::L1Position)),
                intake->setState(Positions::L1Position), grabber->setState(Positions::L1Position),
                climber->setState(Positions::L1Position));
    }}, {Positions::CoralHold, Positions::L2Front, [this]() {
        return grabber->isCoralIn() && (alignManager->getHeading() == Heading::Front)
                && (oprtr->B().Get() || console->Button(12).Get() || console->Button(5).Get());
    }, [this]() {
        return frc2::cmd::Parallel(

        frc2::cmd::Sequence(arm->setState(Positions::L2Front, Heading::Front), elevator->setState(Positions::L2Front)),
                intake->setState(Positions::L2Front), grabber->setState(Positions::L2Front),
                climber->setState(Positions::L2Front));
    }}, {Positions::CoralHold, Positions::L3Front, [this]() {
        return grabber->isCoralIn() && (alignManager->getHeading() == Heading::Front)
                && (oprtr->B().Get() || console->Button(7).Get() || console->Button(8).Get());
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L3Front, Heading::Front),
                elevator->setState(Positions::L3Front), intake->setState(Positions::L3Front),
                grabber->setState(Positions::L3Front), climber->setState(Positions::L3Front));
    }}, {Positions::CoralHold, Positions::L4Front, [this]() {
        return grabber->isCoralIn() && (alignManager->getHeading() == Heading::Front)
                && (oprtr->B().Get() || console->Button(10).Get() || console->Button(11).Get());
    }, [this]() {
        return frc2::cmd::Parallel(

        arm->setState(Positions::L4Front, Heading::Front), elevator->setState(Positions::L4Front)

        , intake->setState(Positions::L4Front), grabber->setState(Positions::L4Front),
                climber->setState(Positions::L4Front));
    }}, {Positions::CoralHold, Positions::L2Back, [ this]() {
        return grabber->isCoralIn() && (alignManager->getHeading() == Heading::Back)
                && (oprtr->B().Get() || console->Button(12).Get() || console->Button(5).Get());
    }, [this]() {
        return frc2::cmd::Parallel(
                frc2::cmd::Sequence(arm->setState(Positions::L2Back, Heading::Back),
                        elevator->setState(Positions::L2Back)), intake->setState(Positions::L2Back),
                grabber->setState(Positions::L2Back), climber->setState(Positions::L2Back));
    }}, {Positions::CoralHold, Positions::L3Back, [this]() {
        return grabber->isCoralIn() && (alignManager->getHeading() == Heading::Back)
                && (oprtr->B().Get() || console->Button(7).Get() || console->Button(8).Get());
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L3Back, Heading::Back),
                elevator->setState(Positions::L3Back), intake->setState(Positions::L3Back),
                grabber->setState(Positions::L3Back), climber->setState(Positions::L3Back));
    }}, {Positions::CoralHold, Positions::L4Back, [this]() {
        return grabber->isCoralIn() && (alignManager->getHeading() == Heading::Back)
                && (oprtr->B().Get() || console->Button(10).Get() || console->Button(11).Get());
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L4Back, Heading::Back),
                elevator->setState(Positions::L4Back), intake->setState(Positions::L4Back),
                grabber->setState(Positions::L4Back), climber->setState(Positions::L4Back));
    }}, {Positions::CoralHold, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::CoralAndAlgae, Positions::CoralHold, [this]() {
        return !grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::CoralHold), arm->setState(Positions::CoralHold),
                elevator->setState(Positions::CoralHold), grabber->setState(Positions::CoralHold),
                climber->setState(Positions::CoralHold));
    }}, {Positions::CoralAndAlgae, Positions::NetPosition, [this]() {
        return grabber->isAlgaeIn() && driver->POVLeft().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::NetPosition), elevator->setState(Positions::NetPosition),
                grabber->setState(Positions::NetPosition), intake->setState(Positions::NetPosition),
                climber->setState(Positions::NetPosition));
    }}, {Positions::CoralAndAlgae, Positions::ProcessorPosition, [this]() {
        return grabber->isAlgaeIn() && (oprtr->LeftBumper().Get() || console->Button(9).Get());
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::ProcessorPosition),
                arm->setState(Positions::ProcessorPosition), elevator->setState(Positions::ProcessorPosition),
                grabber->setState(Positions::ProcessorPosition), climber->setState(Positions::ProcessorPosition));
    }}, {Positions::CoralAndAlgae, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::AlgaeHold, Positions::NetPosition, [this]() {
        return grabber->isAlgaeIn() && driver->POVLeft().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::NetPosition), elevator->setState(Positions::NetPosition),
                grabber->setState(Positions::NetPosition), intake->setState(Positions::NetPosition),
                climber->setState(Positions::NetPosition));
    }}, {Positions::AlgaeHold, Positions::ProcessorPosition, [this]() {
        return grabber->isAlgaeIn() && (oprtr->LeftBumper().Get() || console->Button(9).Get());
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::ProcessorPosition),
                arm->setState(Positions::ProcessorPosition), elevator->setState(Positions::ProcessorPosition),
                grabber->setState(Positions::ProcessorPosition), climber->setState(Positions::ProcessorPosition));
    }}, {Positions::AlgaeHold, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::L2Front, Positions::FrontConfirm, [this]() {
        return grabber->isCoralIn() && driver->RightBumper().Get()
                && (oprtr->B().Get() || console->Button(12).Get() || console->Button(5).Get())
                && alignManager->getHeading() == Heading::Front;
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L2FrontConfirm, Heading::Front),
                elevator->setState(Positions::L2FrontConfirm), intake->setState(Positions::L2FrontConfirm),
                grabber->setState(Positions::L2FrontConfirm), climber->setState(Positions::L2FrontConfirm));
    }}, {Positions::L3Front, Positions::FrontConfirm, [this]() {
        return grabber->isCoralIn() && driver->RightBumper().Get()
                && (oprtr->X().Get() || console->Button(7).Get() || console->Button(8).Get())
                && alignManager->getHeading() == Heading::Front;
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L3FrontConfirm, Heading::Front),
                elevator->setState(Positions::L3FrontConfirm), intake->setState(Positions::L3FrontConfirm),
                grabber->setState(Positions::L3FrontConfirm), climber->setState(Positions::L3FrontConfirm));
    }}, {Positions::L4Front, Positions::FrontConfirm, [this]() {
        return grabber->isCoralIn() && driver->RightBumper().Get()
                && (oprtr->Y().Get() || console->Button(10).Get() || console->Button(11).Get())
                && alignManager->getHeading() == Heading::Front;
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L4FrontConfirm, Heading::Front),
                elevator->setState(Positions::L4FrontConfirm), intake->setState(Positions::L4FrontConfirm),
                grabber->setState(Positions::L4FrontConfirm), climber->setState(Positions::L4FrontConfirm));
    }},

    {Positions::L2Back, Positions::BackConfirm, [this]() {
        return grabber->isCoralIn() && driver->RightBumper().Get()
                && (oprtr->B().Get() || console->Button(12).Get() || console->Button(5).Get())
                && alignManager->getHeading() == Heading::Back;
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L2BackConfirm, Heading::Back),
                elevator->setState(Positions::L2BackConfirm), intake->setState(Positions::L2BackConfirm),
                grabber->setState(Positions::L2BackConfirm), climber->setState(Positions::L2BackConfirm));
    }}, {Positions::L3Back, Positions::BackConfirm, [this]() {
        return grabber->isCoralIn() && driver->RightBumper().Get()
                && (oprtr->X().Get() || console->Button(7).Get() || console->Button(8).Get())
                && alignManager->getHeading() == Heading::Back;
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L3BackConfirm, Heading::Back),
                elevator->setState(Positions::L3BackConfirm), intake->setState(Positions::L3BackConfirm),
                grabber->setState(Positions::L3BackConfirm), climber->setState(Positions::L3BackConfirm));
    }}, {Positions::L4Back, Positions::BackConfirm, [this]() {
        return grabber->isCoralIn() && driver->RightBumper().Get()
                && (oprtr->Y().Get() || console->Button(10).Get() || console->Button(11).Get())
                && alignManager->getHeading() == Heading::Front;
    }, [this]() {
        return frc2::cmd::Parallel(arm->setState(Positions::L4BackConfirm, Heading::Back),
                elevator->setState(Positions::L4BackConfirm), intake->setState(Positions::L4BackConfirm),
                grabber->setState(Positions::L4BackConfirm), climber->setState(Positions::L4BackConfirm));
    }},

    {Positions::NetPosition, Positions::NetConfirm, [this]() {
        return grabber->isAlgaeIn() && driver->RightBumper().Get();
    }, [this]() {
        return frc2::cmd::Sequence(grabber->setState(Positions::NetConfirm), arm->setState(Positions::NetConfirm),
                elevator->setState(Positions::NetConfirm), intake->setState(Positions::NetConfirm),
                climber->setState(Positions::NetConfirm));
    }}, {Positions::NetPosition, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::ProcessorPosition, Positions::ProcessorConfirm, [this]() {
        return grabber->isAlgaeIn() && driver->RightBumper().Get();
    }, [this]() {
        return frc2::cmd::Sequence(grabber->setState(Positions::ProcessorConfirm),
                intake->setState(Positions::ProcessorConfirm), arm->setState(Positions::ProcessorConfirm),
                elevator->setState(Positions::ProcessorConfirm), climber->setState(Positions::ProcessorConfirm));
    }}, {Positions::ProcessorPosition, Positions::EndPosition, [this]() {
        return console->Button(4).Get() || oprtr->Back().Get();
    }, [this]() {
        return frc2::cmd::Sequence(arm->setState(Positions::EndPosition), elevator->setState(Positions::EndPosition),
                intake->setState(Positions::EndPosition), grabber->setState(Positions::EndPosition),
                climber->setState(Positions::EndPosition));
    }},

    {Positions::FrontConfirm, Positions::SustainedPosition, [this]() {
        return !grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }},

    {Positions::BackConfirm, Positions::SustainedPosition, [this]() {
        return !grabber->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }},

    {Positions::NetConfirm, Positions::SustainedPosition, [this]() {
        return !grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::NetConfirm, Positions::CoralHold, [this]() {
        return !grabber->isAlgaeIn() && intake->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::CoralHold), arm->setState(Positions::CoralHold),
                elevator->setState(Positions::CoralHold), grabber->setState(Positions::CoralHold),
                climber->setState(Positions::CoralHold));
    }},

    {Positions::ProcessorConfirm, Positions::SustainedPosition, [this]() {
        return !grabber->isAlgaeIn();
    }, [this]() {
        return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
                arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
                grabber->setState(Positions::SustainedPosition), climber->setState(Positions::SustainedPosition));
    }}, {Positions::ProcessorConfirm, Positions::CoralHold, [this]() {
        return !grabber->isAlgaeIn() && intake->isCoralIn();
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::CoralHold), arm->setState(Positions::CoralHold),
                elevator->setState(Positions::CoralHold), grabber->setState(Positions::CoralHold),
                climber->setState(Positions::CoralHold));
    }}, {Positions::EndPosition, Positions::InitialPosition, [this]() {
        return endToInitial->Get() /*Boton en la driver station(tipo los offsets)*/;
    }, [this]() {
        return frc2::cmd::Sequence(intake->setState(Positions::InitialPosition),
                arm->setState(Positions::InitialPosition), elevator->setState(Positions::InitialPosition),
                grabber->setState(Positions::InitialPosition), climber->setState(Positions::InitialPosition));
    }}

    };

};

