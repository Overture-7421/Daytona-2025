#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ClimberConstants {

    constexpr static const units::degree_t ClimberRangeError = 1.0_deg;

    constexpr static const units::degree_t ClosedPosition = 0_deg;
    constexpr static const units::degree_t OpenPosition = 0_deg;

    constexpr static const units::turns_per_second_t ClimberCruiseVelocity = 0.0_tps;
    constexpr static const units::turns_per_second_squared_t ClimberCruiseAcceleration = 0.0_tr_per_s_sq;

    constexpr static const double ClimberEncoderOffset = 0;
    constexpr static const double ClimberSensorToMechanism = 0;

    constexpr static const double ClimberMotorId = 22;

    constexpr static const OverTalonFXConfig ClimberConfig() {
        OverTalonFXConfig climberConfig;
        climberConfig.MotorId = ClimberMotorId;
        climberConfig.NeutralMode = ControllerNeutralMode::Brake;
        climberConfig.Inverted = true;
        climberConfig.useFOC = true;

        climberConfig.CurrentLimit = 20_A;
        climberConfig.StatorCurrentLimit = 120_A;
        climberConfig.TriggerThreshold = 30_A;
        climberConfig.TriggerThresholdTime = 0.5_s;
        climberConfig.ClosedLoopRampRate = 0.05_s;
        climberConfig.PIDConfigs.WithKP(0.0).WithKI(0.0);

        return climberConfig;
    }

};
