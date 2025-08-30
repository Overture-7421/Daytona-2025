// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "EmergencyCommand.h"

frc2::CommandPtr EmergencyCommand(StateManager *stateManager, Intake *intake, Arm *arm, Elevator *elevator,
        Grabber *grabber, Climber *climber) {
    return frc2::cmd::Sequence(elevator->setState(Positions::SustainedPosition),
            arm->setState(Positions::SustainedPosition), intake->setState(Positions::SustainedPosition),
            frc2::cmd::Parallel(stateManager->setStateOverride(), grabber->setState(Positions::SustainedPosition),
                    climber->setState(Positions::SustainedPosition))

                    );

}
