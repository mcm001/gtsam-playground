/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <utility>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>

#include "TagDetection.h"
#include "gtsam_tag_map.h"

namespace wpical {
using KeyframeMap = std::map<gtsam::Key, std::vector<TagDetection>>;

struct CalResult {
  gtsam::Values result;

  std::map<int32_t, gtsam::Matrix> tagPoseCovariances;
  std::map<gtsam::Key, gtsam::Matrix> cameraPoseCovariances;

  frc::AprilTagFieldLayout optimizedLayout;
};

// note that we expect the pixel locations to be -undistorted- here
CalResult OptimizeLayout(
    const GtsamApriltagMap &tagLayoutGuess, const KeyframeMap &keyframes,
    gtsam::Cal3_S2 cal,
    const std::map<int32_t, std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>>
        &fixedTags,
    const gtsam::SharedNoiseModel cameraNoise);
} // namespace wpical
