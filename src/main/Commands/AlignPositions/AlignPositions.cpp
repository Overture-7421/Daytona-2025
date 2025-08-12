// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/AlignPositions/AlignPositions.h"

frc2::CommandPtr leftAlignPos(AlignManager *alignManager) {
    return alignManager->AlignToPose(ReefSide::Left);
}

frc2::CommandPtr rightAlignPos(AlignManager *alignManager) {
    return alignManager->AlignToPose(ReefSide::Right);
}

frc2::CommandPtr algaeAlignPos(AlignManager *alignManager) {
    return alignManager->AlignToPose(ReefSide::Algae);
}
