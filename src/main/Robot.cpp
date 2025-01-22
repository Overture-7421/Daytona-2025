// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {

#ifndef __FRC_ROBORIO__
    simMotorManager.Init( { {8, "Reefscape2025/motors/back_right_drive"}, {6, "Reefscape2025/motors/back_left_drive"}, {
            2, "Reefscape2025/motors/front_left_drive"}, {4, "Reefscape2025/motors/front_right_drive"},

    {7, "Reefscape2025/motors/back_right_rotation"}, {5, "Reefscape2025/motors/back_left_rotation"}, {1,
            "Reefscape2025/motors/front_left_rotation"}, {3, "Reefscape2025/motors/front_right_rotation"}, {13,
            "Reefscape2025/motors/intake"}, {14, "Reefscape2025/motors/elevator"}, {15, "Reefscape2025/motors/arm"}, {
            18, "Reefscape2025/motors/arm_rotator"}, {20, "Reefscape2025/motors/climber"}});

    simPigeonManager.Init("Reefscape2025/imu");
    frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
    simPhotonVisionManager.Init(tagLayout);

    simCANCoderManager.Init( { {11, "Reefscape2025/cancoders/back_right_cancoder"}, {10,
            "Reefscape2025/cancoders/back_left_cancoder"}, {9, "Reefscape2025/cancoders/front_left_cancoder"}, {12,
            "Reefscape2025/cancoders/front_right_cancoder"}, {17, "Reefscape2025/cancoders/arm"}, {19,
            "Reefscape2025/cancoders/arm_rotor"}});

#endif

    AddPeriodic([&] {
        frc2::CommandScheduler::GetInstance().Run();
    }, RobotConstants::LoopTime, RobotConstants::TimingOffset);
}

void Robot::RobotPeriodic() {
    m_container.UpdateTelemetry();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::DisabledExit() {
}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {
    arm.setToAngle(30_deg, 0_deg);
}

void Robot::TeleopExit() {
}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {
}

void Robot::TestExit() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
