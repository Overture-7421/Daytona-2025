// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Enums/Positions.h"
#include <functional>
#include <frc2/command/CommandPtr.h>

struct Transitions {
    const Positions currentState;
    const Positions nextState;
    std::function<bool()> check;
    std::function<frc2::CommandPtr()> commandGenerator;

    // Constructor for command generators
    Transitions(Positions current, Positions next, std::function<bool()> checkFunc,
            std::function<frc2::CommandPtr()> cmdGen) : currentState(current), nextState(next), check(checkFunc), commandGenerator(
            cmdGen) {
    }
};
