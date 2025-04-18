// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Commands/UtilityFunctions/UtilityFunctions.h"
#include <map>
#include "Enums/ReefSide.h"
#include "Commands/UtilityFunctions/ReefOffset.h"
#include <OvertureLib/Gamepads/OverXboxController/OverXboxController.h>

//MOVER DERECHA es mas POSITIVOS / MOVER IZQUIERDA es mas NEGATIVOS 
//ATRAS es mas POSITIVOS / ADELANTE es mas NEGATIVOS
//GIRAR IZQUIERDA mas POSITIVOS / GIRAR DERECHA mas NEGATIVOS

//IZQUIERDA, DERECHA, ADELANTE/ATRAS, ROTACION
static const ReefOffset defaultReefOffset{ 0.13_m, 0.475_m, 0.56_m, 180.0_deg };

static std::map<ReefLocation, ReefOffset> alignPositionsMap;

static const std::map<ReefLocation, ReefOffset> alignInRed = {};

static const std::map<ReefLocation, ReefOffset> alignInBlue = {};

frc2::CommandPtr leftAlignPos(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout, OverXboxController* driver);
frc2::CommandPtr rightAlignPos(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout, OverXboxController* driver);

