// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "UtilityFunctions.h"

ReefLocation findClosestReefLocation(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {

    const std::vector<std::pair<ReefLocation, frc::Pose2d>> blueReefLocations = { {ReefLocation::A,
            tagLayout->GetTagPose(17).value().ToPose2d()}, {ReefLocation::B,
            tagLayout->GetTagPose(18).value().ToPose2d()}, {ReefLocation::C,
            tagLayout->GetTagPose(19).value().ToPose2d()}, {ReefLocation::D,
            tagLayout->GetTagPose(20).value().ToPose2d()}, {ReefLocation::E,
            tagLayout->GetTagPose(21).value().ToPose2d()}, {ReefLocation::F,
            tagLayout->GetTagPose(22).value().ToPose2d()}};

    const std::vector<std::pair<ReefLocation, frc::Pose2d>> redReefLocations = { {ReefLocation::A,
            tagLayout->GetTagPose(8).value().ToPose2d()},
            {ReefLocation::B, tagLayout->GetTagPose(7).value().ToPose2d()}, {ReefLocation::C,
                    tagLayout->GetTagPose(6).value().ToPose2d()}, {ReefLocation::D,
                    tagLayout->GetTagPose(11).value().ToPose2d()}, {ReefLocation::E,
                    tagLayout->GetTagPose(10).value().ToPose2d()}, {ReefLocation::F,
                    tagLayout->GetTagPose(9).value().ToPose2d()}};

    std::vector<std::pair<ReefLocation, units::meter_t>> distancesToReefLocations;
    distancesToReefLocations.reserve(6);

    const std::vector<std::pair<ReefLocation, frc::Pose2d>> *reefLocations = &blueReefLocations;

    if (isRedAlliance()) {
        reefLocations = &redReefLocations;
    }

    for (auto location : *reefLocations) {
        distancesToReefLocations.push_back(std::pair {location.first, getDistanceToChassis(chassis, location.second)});
    }

    std::sort(distancesToReefLocations.begin(), distancesToReefLocations.end(), [](auto a, auto b) {
        return a.second < b.second;
    });

    return distancesToReefLocations.front().first;
}

StationLocation findClosestStationLocation(Chassis *chassis, frc::AprilTagFieldLayout *tagLayout) {

    const std::vector<std::pair<StationLocation, frc::Pose2d>> blueStationLocations = { {StationLocation::Left,
            tagLayout->GetTagPose(13).value().ToPose2d()}, {StationLocation::Right,
            tagLayout->GetTagPose(12).value().ToPose2d()}};

    const std::vector<std::pair<StationLocation, frc::Pose2d>> redStationLocations = { {StationLocation::Left,
            tagLayout->GetTagPose(1).value().ToPose2d()}, {StationLocation::Right,
            tagLayout->GetTagPose(2).value().ToPose2d()}};

    std::vector<std::pair<StationLocation, units::meter_t>> distancesToStationLocations;
    distancesToStationLocations.reserve(6);

    const std::vector<std::pair<StationLocation, frc::Pose2d>> *stationLocations = &blueStationLocations;

    if (isRedAlliance()) {
        stationLocations = &redStationLocations;
    }

    for (auto location : *stationLocations) {
        distancesToStationLocations.push_back(
                std::pair {location.first, getDistanceToChassis(chassis, location.second)});
    }

    std::sort(distancesToStationLocations.begin(), distancesToStationLocations.end(), [](auto a, auto b) {
        return a.second < b.second;
    });

    return distancesToStationLocations.front().first;

}
