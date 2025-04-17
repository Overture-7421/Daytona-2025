#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct ClimberConstants {
    constexpr static const double ArmSensorToMechanism = 265.84615385;
    //constexpr static const units::meter_t Diameter = 0.0127_m; //0.5in

    constexpr static const units::turns_per_second_t ArmCruiseVelocity = 30.0_tps; //6
    constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 100_tr_per_s_sq; //13

    constexpr static const double RightArmMotorId = 22;

    constexpr static const units::degree_t ClosedPosition = -50_deg;
    constexpr static const units::degree_t OpenPosition = 75_deg; //Cuidado

    constexpr static const OverTalonFXConfig RightConfig() {
        OverTalonFXConfig rightConfig;
        rightConfig.MotorId = RightArmMotorId;
        rightConfig.NeutralMode = ControllerNeutralMode::Brake;
        rightConfig.Inverted = true;
        rightConfig.useFOC = true;

        rightConfig.CurrentLimit = 20_A;
        rightConfig.StatorCurrentLimit = 120_A;
        rightConfig.TriggerThreshold = 30_A;
        rightConfig.TriggerThresholdTime = 0.5_s;
        rightConfig.ClosedLoopRampRate = 0.05_s;
        rightConfig.PIDConfigs.WithKP(150.0).WithKI(10); //60

        return rightConfig;
    }

};
