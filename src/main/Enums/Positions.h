#pragma once

enum class Positions {
    InitialPosition,

    SustainedPosition,

    Intake,

    IntakeCoralStation,

    AlgaeHighReef,

    AlgaeLowReef,

    AlgaeGround,

    L1Position,

    L1Confirm,

    CoralHold,

    CoralAndAlgae,

    AlgaeHold,

    L2Front, L3Front, L4Front,

    L2FrontConfirm, L3FrontConfirm, L4FrontConfirm,

    L2Back, L3Back, L4Back,

    L2BackConfirm, L3BackConfirm, L4BackConfirm,

    NetPosition, NetConfirm,

    ProcessorPosition, ProcessorConfirm, FrontConfirm, BackConfirm,

    EndPosition, Through, AlgaeTension, CoralSpit, SustainToL1, L4FrontAuto, L4FrontAutoConfirm
};

