// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {

#ifndef __FRC_ROBORIO__
	simMotorManager.Init({
	  {2, "Offseason 2024/motors/back_right_drive"},
	  {4, "Offseason 2024/motors/back_left_drive"},
	  {6, "Offseason 2024/motors/front_left_drive"},
	  {8, "Offseason 2024/motors/front_right_drive"},

	  {1, "Offseason 2024/motors/back_right_rotation"},
	  {3, "Offseason 2024/motors/back_left_rotation"},
	  {5, "Offseason 2024/motors/front_left_rotation"},
	  {7, "Offseason 2024/motors/front_right_rotation"}
		});

	simPigeonManager.Init("Offseason 2024/imu");

	simPhotonVisionManager.Init(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField));

	simCANCoderManager.Init({
	  {11, "Offseason 2024/cancoders/back_right_cancoder"},
	  {10, "Offseason 2024/cancoders/back_left_cancoder"},
	  {9, "Offseason 2024/cancoders/front_left_cancoder"},
	  {12, "Offseason 2024/cancoders/front_right_cancoder"}
		});

#endif

	AddPeriodic([&] {
		frc2::CommandScheduler::GetInstance().Run();
	}, RobotConstants::LoopTime, RobotConstants::TimingOffset);
}

void Robot::RobotPeriodic() {
	m_container.UpdateTelemetry();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
	m_autonomousCommand = m_container.GetAutonomousCommand();

	if (m_autonomousCommand) {
		m_autonomousCommand->Schedule();
	}
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
	if (m_autonomousCommand) {
		m_autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
	frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
