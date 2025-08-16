// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Grabber/Grabber.h"
#include <map>
#include "Enums/Positions.h"

struct GrabberConstants {
public:

inline static const std::map<Positions, units::volt_t> GrabberVoltage = {
        {Positions::AlgaeGround, 8.0_V}, 
        {Positions::AlgaeHold, 2.0_V},
        {Positions::AlgaeHighReef,8.0_V},
        {Positions::AlgaeLowReef, 8.0_V},
        {Positions::CoralandAlgae, 8.0_V},
        {Positions::CoralHold, 0.0_V},
        {Positions::InitialPosition, 0.0_V},
        {Positions::Intake, 4.0_V},
        {Positions::IntakeCoralStation, 0.0_V},
        {Positions::L1Confirm, 0.0_V},
        {Positions::L1Position, 0.0_V},
        {Positions::L2Back, 0.0_V},
        {Positions::L2BackConfirm, 0.0_V},
        {Positions::L2Front, 0.0_V},
        {Positions::L2FrontConfirm, 0.0_V},
        {Positions::L3Back, 0.0_V},
        {Positions::L3BackConfirm, 0.0_V},
        {Positions::L3Front, 0.0_V},
        {Positions::L3FrontConfirm, 0.0_V},
        {Positions::L4Back, 0.0_V},
        {Positions::L4BackConfirm, 0.0_V},
        {Positions::L4Front, 0.0_V},
        {Positions::L4FrontConfirm, 0.0_V},
        {Positions::NetPosition,2.0_V},
        {Positions::NetConfirm, -4.0_V},
        {Positions::ProcessorPosition, 2.0_V},
        {Positions::ProcessorConfirm, -4.0_V},
        {Positions::SustainedPosition, 0.0_V}        
    };

    constexpr static const double CoralDetectionCurrent = 35.0; //To be defined
    constexpr static const double AlgaeDetectionCurrent = 20.0; //To be defined

    //Configuration for the Motor
    constexpr static const OverTalonFXConfig GrabberConfig() {
        OverTalonFXConfig grabberConfig;
        grabberConfig.MotorId = 28;
        grabberConfig.NeutralMode = ControllerNeutralMode::Brake;
        grabberConfig.Inverted = true;
        grabberConfig.CurrentLimit = 25_A;
        grabberConfig.StatorCurrentLimit = 120_A;
        grabberConfig.TriggerThreshold = 40_A;
        grabberConfig.TriggerThresholdTime = 0.5_s;
        grabberConfig.ClosedLoopRampRate = 0.0_s;
        grabberConfig.OpenLoopRampRate = 0.05_s;

        return grabberConfig;
    };

};
