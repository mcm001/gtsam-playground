#pragma once

#include <optional>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include <units/time.h>
#include "TagDetection.h"
#include <vector>

using CameraMatrix = Eigen::Matrix<double, 3, 3>;
using DistortionMatrix = Eigen::Matrix<double, 8, 1>;

namespace photon {
    std::optional<frc::Pose3d> MultiTagOnRioStrategy(
        std::vector<TagDetection> result,
        frc::AprilTagFieldLayout aprilTags,
        std::optional<CameraMatrix> camMat,
        std::optional<DistortionMatrix> distCoeffs);
}