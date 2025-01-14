#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct Constants {

  constexpr static const double ArmSensorToMechanism = 0.0;
  constexpr static const double HookSensorToMechanism = 0.0;


  constexpr static const units::turns_per_second_t ArmCruiseVelocity = 5.0_tps;
  constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 15_tr_per_s_sq;
  constexpr static const units::turns_per_second_t HookCruiseVelocity = 5.0_tps;
  constexpr static const units::turns_per_second_squared_t HookCruiseAcceleration = 15_tr_per_s_sq;

  constexpr static const double ArmMotorId = 20;
  constexpr static const double HookMotorId = 29;
  

  constexpr static const OverTalonFXConfig armConfig(){
    OverTalonFXConfig armConfig;
    armConfig.MotorId = ArmMotorId;
    armConfig.NeutralMode = ControllerNeutralMode::Brake;
    armConfig.useFOC = true;

    armConfig.ClosedLoopRampRate = 0.01_s;
    armConfig.PIDConfigs.WithKP(0.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

    return armConfig;
  }

  constexpr static const OverTalonFXConfig hookConfig(){
    OverTalonFXConfig hookConfig;
    hookConfig.MotorId = HookMotorId;
    hookConfig.NeutralMode = ControllerNeutralMode::Brake;
    hookConfig.useFOC = true;

    hookConfig.ClosedLoopRampRate = 0.01_s;
    hookConfig.PIDConfigs.WithKP(0.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

    return hookConfig;
  }

};