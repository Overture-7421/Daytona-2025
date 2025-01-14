#pragma once

#include <units/length.h>
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct ElevatorConstants{
    constexpr static const double LowerGearRatio = 0.0;
    constexpr static const double LowerSensorToMechanism = 0.0;
    constexpr static const units::meter_t diameter = 0.0_m;

    constexpr static const OverTalonFXConfig rightConfig(){
    OverTalonFXConfig right;
    right.MotorId = 10;
    right.NeutralMode = ControllerNeutralMode::Brake;
    right.Inverted = true;
    right.useFOC = true;
    right.PIDConfigs.WithKP(0);
    right.CurrentLimit = 0_A;
    right.StatorCurrentLimit = 0_A;
    right.TriggerThreshold = 0_A;
    right.TriggerThresholdTime = 0_s;
    right.ClosedLoopRampRate = 0_s;
    right.OpenLoopRampRate = 0_s;

    return right;
    }

    constexpr static const OverTalonFXConfig leftConfig(){
    OverTalonFXConfig left;
    left.MotorId = 9;
    left.NeutralMode = ControllerNeutralMode::Brake;
    left.Inverted = false;
    left.useFOC = true;
    left.PIDConfigs.WithKP(0);
    left.CurrentLimit = 0_A;
    left.StatorCurrentLimit = 0_A;
    left.TriggerThreshold = 0_A;
    left.TriggerThresholdTime = 0_s;
    left.ClosedLoopRampRate = 0_s;
    left.OpenLoopRampRate = 0_s;

    return left;
    
    }
};

