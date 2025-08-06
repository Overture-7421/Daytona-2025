// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//.

#pragma once
#include <units/voltage.h>
#include "Subsystems/Grabber/Grabber.h"

struct GrabberConstants {
public:

    //Voltage for corals
    constexpr static const units::volt_t GrabCoral = 4.0_V;
    constexpr static const units::volt_t SpitCoral = -1.5_V;

    //Voltage for Algaes
    constexpr static const units::volt_t GrabAlgae = 8.0_V;
    constexpr static const units::volt_t HoldAlgae = 2_V;
    constexpr static const units::volt_t SpitAlgae = -4.0_V;

    constexpr static const units::volt_t StopIntake = 0.0_V;

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
    }

};
