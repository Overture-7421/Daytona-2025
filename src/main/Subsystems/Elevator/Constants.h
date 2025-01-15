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
    constexpr static const units::meter_t Diameter = 0.0_m;

    constexpr static const OverTalonFXConfig RightConfig(){
    OverTalonFXConfig right;
    right.MotorId = 10;
    right.NeutralMode = ControllerNeutralMode::Brake;
    right.Inverted = true;
    right.useFOC = true;
    right.PIDConfigs.WithKP(0);
    right.CurrentLimit = 30_A;
    right.StatorCurrentLimit = 30_A;
    right.TriggerThreshold = 90_A;
    right.TriggerThresholdTime = 1_s;
    right.ClosedLoopRampRate = 0.05_s;
    right.OpenLoopRampRate = 0.05_s;

    return right;
    }

    constexpr static const OverTalonFXConfig LeftConfig(){
    OverTalonFXConfig left;
    left.MotorId = 9;
    left.NeutralMode = ControllerNeutralMode::Brake;
    left.Inverted = false;
    left.useFOC = true;
    left.PIDConfigs.WithKP(0);
    left.CurrentLimit = 30_A;
    left.StatorCurrentLimit = 30_A;
    left.TriggerThreshold = 90_A;
    left.TriggerThresholdTime = 1_s;
    left.ClosedLoopRampRate = 0.05_s;
    left.OpenLoopRampRate = 0.05_s;

    return left;
    
    }
};