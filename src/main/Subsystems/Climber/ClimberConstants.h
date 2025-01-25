#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ClimberConstants {
    constexpr static const double ArmSensorToMechanism = 200.0;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 6.0_tps;
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 13_tr_per_s_sq;

    constexpr static const double RightArmMotorId = 20;
    constexpr static const double LeftArmMotorId = 21;

    constexpr static const units::degree_t ClosedPosition = 0_deg;
    constexpr static const units::degree_t MiddlePosition = 20_deg; //The functionality of this position is still to be defined.. It is thought to be a reciving position for the cage.
    constexpr static const units::degree_t OpenPosition = 40_deg;

    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig rightConfig;
        rightConfig.MotorId = RightArmMotorId;
        rightConfig.NeutralMode = ControllerNeutralMode::Brake;
        rightConfig.Inverted = false;
        rightConfig.useFOC = true;
        rightConfig.CurrentLimit = 30_A;
        rightConfig.StatorCurrentLimit = 30_A;
        rightConfig.TriggerThreshold = 90_A;
        rightConfig.TriggerThresholdTime = 1_s;
        rightConfig.ClosedLoopRampRate = 0.05_s;
        rightConfig.OpenLoopRampRate = 0.05_s;
        rightConfig.PIDConfigs.WithKP(15.0);

        return rightConfig;
    }

    constexpr static const OverTalonFXConfig LeftConfig() {
        OverTalonFXConfig leftConfig;
        leftConfig.MotorId = LeftArmMotorId;
        leftConfig.NeutralMode = ControllerNeutralMode::Brake;
        leftConfig.Inverted = true;
        leftConfig.useFOC = true;
        leftConfig.CurrentLimit = 30_A;
        leftConfig.StatorCurrentLimit = 30_A;
        leftConfig.TriggerThreshold = 90_A;
        leftConfig.TriggerThresholdTime = 1_s;
        leftConfig.ClosedLoopRampRate = 0.05_s;
        leftConfig.OpenLoopRampRate = 0.05_s;
        leftConfig.PIDConfigs.WithKP(15.0);

        return leftConfig;
    }

};
