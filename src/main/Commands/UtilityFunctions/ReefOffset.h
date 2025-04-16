// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <math.h>

struct ReefOffset {
    units::meter_t leftOffset;
    units::meter_t rightOffset;
    units::meter_t xOffset;
    units::degree_t headingOffset;
};
