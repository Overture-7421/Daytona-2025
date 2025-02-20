#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ClimberConstants {
    constexpr static const double ArmSensorToMechanism = 265.8461538;

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 15.0_tps; //6
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 12_tr_per_s_sq; //13

    constexpr static const double RightArmMotorId = 22;

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
        rightConfig.PIDConfigs.WithKP(0.0); //15

        return rightConfig;
    }

};
