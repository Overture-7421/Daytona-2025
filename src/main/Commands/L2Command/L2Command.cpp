// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "L2Command.h"

frc2::CommandPtr L2Command(Arm *arm, Elevator *elevator, Intake *intake) {
    return frc2::cmd::Select < IntakeStates
            > ([intake] {
                return intake->getState();
            },
            std::pair {IntakeStates::HoldCoral, frc2::cmd::Parallel(
                    elevator->setElevatorCommand(ElevatorConstants::L2Position),
                    ArmMotion(elevator, arm, ArmConstants::ArmL2Reef, ArmConstants::WristL2Reef,
                            ElevatorConstants::L2Position).ToPtr())}

            );

}
