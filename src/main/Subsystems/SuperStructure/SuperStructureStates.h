// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

enum SuperStructureStates {
    HoldCoral, HoldAlgae,

    EnterCoralStation,

    EnterCoralGround,

    EnterAlgaeGround,

    EnterLowAlgae, EnterHighAlgae,

    SpitAlgae, SpitCoral

};

enum SuperStructureScoringStates {
    L1, L2, L3, L4, DontScore, ProcessorState, NetState, SpitAlgaeState, HoldBadCoral
};
