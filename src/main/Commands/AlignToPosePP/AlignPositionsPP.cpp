// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignPositionsPP.h"

frc2::CommandPtr leftAlignPP(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout, AprilTags* frontRightCamera) {
	return frc2::cmd::Select < ReefLocation
	>([chassis, tagLayout]() {
		return findClosestReefLocation(chassis, tagLayout);
	},
		std::pair{ ReefLocation::A, pathplanner::AutoBuilder::pathfindToPoseFlipped(aPose, constraints,0.5_mps) },
		std::pair{ ReefLocation::B, pathplanner::AutoBuilder::pathfindToPoseFlipped(bPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::C, pathplanner::AutoBuilder::pathfindToPoseFlipped(cPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::D, pathplanner::AutoBuilder::pathfindToPoseFlipped(dPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::E, pathplanner::AutoBuilder::pathfindToPoseFlipped(ePose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::F, pathplanner::AutoBuilder::pathfindToPoseFlipped(fPose, constraints, 0.5_mps) }
	).AndThen(AlignToPosePP(chassis, left, frontRightCamera).ToPtr());
}

frc2::CommandPtr centerAlignPP(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout, AprilTags* frontRightCamera) {
	return frc2::cmd::Select < ReefLocation
	>([chassis, tagLayout]() {
		return findClosestReefLocation(chassis, tagLayout);
	},
		std::pair{ ReefLocation::A, pathplanner::AutoBuilder::pathfindToPoseFlipped(aPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::B, pathplanner::AutoBuilder::pathfindToPoseFlipped(bPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::C, pathplanner::AutoBuilder::pathfindToPoseFlipped(cPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::D, pathplanner::AutoBuilder::pathfindToPoseFlipped(dPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::E, pathplanner::AutoBuilder::pathfindToPoseFlipped(ePose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::F, pathplanner::AutoBuilder::pathfindToPoseFlipped(fPose, constraints, 0.5_mps) }
	).AndThen(AlignToPosePP(chassis, center, frontRightCamera).ToPtr());
}

frc2::CommandPtr rightAlignPP(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout, AprilTags* frontRightCamera) {
	return frc2::cmd::Select < ReefLocation
	>([chassis, tagLayout]() {
		return findClosestReefLocation(chassis, tagLayout);
	},
		std::pair{ ReefLocation::A, pathplanner::AutoBuilder::pathfindToPoseFlipped(aPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::B, pathplanner::AutoBuilder::pathfindToPoseFlipped(bPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::C, pathplanner::AutoBuilder::pathfindToPoseFlipped(cPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::D, pathplanner::AutoBuilder::pathfindToPoseFlipped(dPose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::E, pathplanner::AutoBuilder::pathfindToPoseFlipped(ePose, constraints, 0.5_mps) },
		std::pair{ ReefLocation::F, pathplanner::AutoBuilder::pathfindToPoseFlipped(fPose, constraints, 0.5_mps) }
	).AndThen(AlignToPosePP(chassis, right, frontRightCamera).ToPtr());
}

frc2::CommandPtr pathFindProcessor() {
	return pathplanner::AutoBuilder::pathfindToPose(
		processorPose,
		constraints,
		0_mps
	);
}

frc2::CommandPtr stationAlignPP(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout) {
	return frc2::cmd::Select < StationLocation >([chassis, tagLayout]() {
		return findClosestStationLocation(chassis, tagLayout);
	},
		std::pair{ StationLocation::Left, AutoBuilder::pathfindToPose(
			stationPoseLeft,
			constraints,
			0_mps
		) },
		std::pair{ StationLocation::Right, AutoBuilder::pathfindToPose(
				stationPoseRight,
				constraints,
				0_mps
			) });
}
