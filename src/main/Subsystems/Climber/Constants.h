#pragma once
#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct Constants {

  constexpr static const double ArmSensorToMechanism = 0.0;

  constexpr static const units::turns_per_second_t ArmCruiseVelocity = 5.0_tps;
  constexpr static const units::turns_per_second_squared_t ArmCruiseAcceleration = 15_tr_per_s_sq;

  constexpr static const double rightArmMotorId = 20;
  constexpr static const double leftArmMotorId = 29;
  

  constexpr static const OverTalonFXConfig RightConfig(){
    OverTalonFXConfig rightConfig;
    rightConfig.MotorId = rightArmMotorId;
    rightConfig.NeutralMode = ControllerNeutralMode::Brake;
    rightConfig.useFOC = true;
    rightConfig.ClosedLoopRampRate = 0.01_s;
    rightConfig.PIDConfigs.WithKP(70.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

    return rightConfig;
  }

  constexpr static const OverTalonFXConfig LeftConfig(){
    OverTalonFXConfig leftConfig;
    leftConfig.MotorId = leftArmMotorId;
    leftConfig.NeutralMode = ControllerNeutralMode::Brake;
    leftConfig.useFOC = true;
    leftConfig.ClosedLoopRampRate = 0.01_s;
    leftConfig.PIDConfigs.WithKP(70.0).WithKI(0.0).WithKD(0.0).WithKS(0.0).WithKG(0.0).WithKV(0.0).WithKA(0.0);

    return leftConfig;
  }

};