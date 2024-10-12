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

#include <gtsam/base/types.h> // Basic types, constants, and compatibility functions
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <optional>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <wpi/json.h>

#include "TagDetection.h"

void from_json(const wpi::json &json, TargetCorner &corner);
void from_json(const wpi::json &json, TagDetection &tag);
std::map<gtsam::Key, std::vector<TagDetection>> ParseFile();
std::optional<gtsam::Pose3> estimateWorldTcam(std::vector<TagDetection> tags,
                                              frc::AprilTagFieldLayout layout,
                                              double fx, double fy, double cx,
                                              double cy);
bool tagWasUsed(std::map<gtsam::Key, std::vector<TagDetection>> tags, int id);

namespace helpers {

std::optional<frc::AprilTag> GetTagPose(frc::AprilTagFieldLayout layoutGuess,
                                        int id);

inline gtsam::Key CameraIdxToKey(int camIdx) {
  return gtsam::symbol_shorthand::C(camIdx);
}
inline gtsam::Key StateNumToKey(int i) { return gtsam::symbol_shorthand::X(i); }
inline gtsam::Key TagIdToKey(int tagId) {
  return gtsam::symbol_shorthand::L(tagId);
}
inline int KeyToTagId(gtsam::Key tagId) {
  return static_cast<int>(tagId - TagIdToKey(0));
}

gtsam::Pose3 TwistToPoseDelta(frc::Twist3d twist);

} // namespace helpers

namespace sfm_mapper {
using Pose3 = gtsam::Pose3;

struct OdomPoseDelta {
  int64_t time;

  // Tags in view
  gtsam::Pose3 poseDelta;

  // Robot state keys for the start and end of this twist
  // gtsam::Key stateFrom;
  // gtsam::Key stateTo;

  // // Best guess at robot pose
  // gtsam::Pose3 poseGuess;
};

struct KeyframeData {
  int64_t time;

  // Camera that saw this
  gtsam::Key cameraIdx;

  // Tags in view
  std::vector<TagDetection> observation;

  // // Robot state key at snapshot time
  // gtsam::Key robotState;
  // // Best guess at robot pose
  // gtsam::Pose3 poseGuess;
};

using OdometryList = std::vector<OdomPoseDelta>;
using KeyframeList = std::vector<KeyframeData>;

/**
 * Stores both input info to the optimizer and outputs from the optimizer
 */
struct OptimizerState {

  // Pose-deltas from odometry
  OdometryList odometryMeasurements;
  // Keyframes from our camera
  KeyframeList keyframes;
};
} // namespace sfm_mapper
