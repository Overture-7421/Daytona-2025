// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "UtilityFunctions.h"

ReefPackage findClosestReefLocation(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {
    const std::vector<ReefPackage> blueReefLocations = { {frc::DriverStation::Alliance::kBlue, ReefLocation::A,
            tagLayout->GetTagPose(17).value().ToPose2d(), AlgaePose::Down}, {frc::DriverStation::Alliance::kBlue,
            ReefLocation::B, tagLayout->GetTagPose(18).value().ToPose2d(), AlgaePose::Up}, {
            frc::DriverStation::Alliance::kBlue, ReefLocation::C, tagLayout->GetTagPose(19).value().ToPose2d(),
            AlgaePose::Down}, {frc::DriverStation::Alliance::kBlue, ReefLocation::D,
            tagLayout->GetTagPose(20).value().ToPose2d(), AlgaePose::Up}, {frc::DriverStation::Alliance::kBlue,
            ReefLocation::E, tagLayout->GetTagPose(21).value().ToPose2d(), AlgaePose::Down}, {
            frc::DriverStation::Alliance::kBlue, ReefLocation::F, tagLayout->GetTagPose(22).value().ToPose2d(),
            AlgaePose::Up}};

    const std::vector<ReefPackage> redReefLocations = { {frc::DriverStation::Alliance::kRed, ReefLocation::A,
            tagLayout->GetTagPose(8).value().ToPose2d(), AlgaePose::Down}, {frc::DriverStation::Alliance::kRed,
            ReefLocation::B, tagLayout->GetTagPose(7).value().ToPose2d(), AlgaePose::Up}, {
            frc::DriverStation::Alliance::kRed, ReefLocation::C, tagLayout->GetTagPose(6).value().ToPose2d(),
            AlgaePose::Down}, {frc::DriverStation::Alliance::kRed, ReefLocation::D,
            tagLayout->GetTagPose(11).value().ToPose2d(), AlgaePose::Up}, {frc::DriverStation::Alliance::kRed,
            ReefLocation::E, tagLayout->GetTagPose(10).value().ToPose2d(), AlgaePose::Down}, {
            frc::DriverStation::Alliance::kRed, ReefLocation::F, tagLayout->GetTagPose(9).value().ToPose2d(),
            AlgaePose::Up}};

    std::vector<std::pair<ReefPackage, units::meter_t>> distancesToReefLocations;
    distancesToReefLocations.reserve(6);

    const std::vector<ReefPackage> *reefLocations = &blueReefLocations;

    if (isRedAlliance()) {
        reefLocations = &redReefLocations;
    }

    for (auto location : *reefLocations) {
        distancesToReefLocations.push_back(std::pair {location, getDistanceToChassis(chassis, location.pose)});
    }

    std::sort(distancesToReefLocations.begin(), distancesToReefLocations.end(), [](auto a, auto b) {
        return a.second < b.second;
    });

    ReefPackage result;
    result.reefLocation = distancesToReefLocations.front().first.reefLocation;
    result.alliance = distancesToReefLocations.front().first.alliance;
    result.pose = distancesToReefLocations.front().first.pose;
    result.algaePose = distancesToReefLocations.front().first.algaePose;

    return result;

}
