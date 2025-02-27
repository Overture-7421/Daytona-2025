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

// frc2::CommandPtr centerAlignPP(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout) {
// 	return frc2::cmd::Select < ReefLocation
// 	>([chassis, tagLayout]() {
// 		return findClosestReefLocation(chassis, tagLayout);
// 	},
// 		std::pair{ ReefLocation::A, AlignToPosePP(chassis, aCenter).ToPtr() }, std::pair{ ReefLocation::B, AlignToPosePP(
// 				chassis, bCenter).ToPtr() }, std::pair{ ReefLocation::C, AlignToPosePP(chassis, cCenter).ToPtr() }, std::pair{
// 				ReefLocation::D, AlignToPosePP(chassis, dCenter).ToPtr() }, std::pair{ ReefLocation::E, AlignToPosePP(
// 				chassis, eCenter).ToPtr() }, std::pair{ ReefLocation::F, AlignToPosePP(chassis, fCenter).ToPtr() });
// }

// frc2::CommandPtr rightAlignPP(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout) {
// 	return frc2::cmd::Select < ReefLocation
// 	>([chassis, tagLayout]() {
// 		return findClosestReefLocation(chassis, tagLayout);
// 	},
// 		std::pair{ ReefLocation::A, AlignToPosePP(chassis, aRight).ToPtr() }, std::pair{ ReefLocation::B, AlignToPosePP(
// 				chassis, bRight).ToPtr() }, std::pair{ ReefLocation::C, AlignToPosePP(chassis, cRight).ToPtr() }, std::pair{
// 				ReefLocation::D, AlignToPosePP(chassis, dRight).ToPtr() }, std::pair{ ReefLocation::E, AlignToPosePP(
// 				chassis, eRight).ToPtr() }, std::pair{ ReefLocation::F, AlignToPosePP(chassis, fRight).ToPtr() });
// }

frc2::CommandPtr pathFindProcessor() {
	return pathplanner::AutoBuilder::pathfindToPoseFlipped(
		processorPose,
		constraints,
		0_mps
	);
}

frc2::CommandPtr stationPos(Chassis* chassis, frc::AprilTagFieldLayout* tagLayout) {
	return frc2::cmd::Select < StationLocation >([chassis, tagLayout]() {
		return findClosestStationLocation(chassis, tagLayout);
	},
		std::pair{ StationLocation::Left, AutoBuilder::pathfindToPoseFlipped(
			stationPoseLeft,
			constraints,
			0_mps
		) },
		std::pair{ StationLocation::Right, AutoBuilder::pathfindToPoseFlipped(
				stationPoseRight,
				constraints,
				0_mps
			) });
}
