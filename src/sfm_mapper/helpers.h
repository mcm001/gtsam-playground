#pragma once

#include <wpi/json.h>
#include "TagDetection.h"
#include <vector>
#include <optional>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <gtsam/base/types.h>  // Basic types, constants, and compatibility functions
#include <gtsam/geometry/Pose3.h>

void from_json(const wpi::json &json, TargetCorner &corner);
void from_json(const wpi::json &json, TagDetection &tag);
std::map<gtsam::Key, std::vector<TagDetection>> ParseFile();
std::optional<gtsam::Pose3> estimateObservationPose(std::vector<TagDetection> tags,
                                     frc::AprilTagFieldLayout layout);
