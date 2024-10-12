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
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/expressions.h>

#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>

#include "TagDetection.h"
#include "helpers.h"
#include "ntcore_cpp_types.h"

namespace sfm_mapper {

class SfmMapper {
public:
  using TimeKeyMap = std::map<int64_t, gtsam::Key>;

  SfmMapper(frc::AprilTagFieldLayout layoutGuess_,
            ::gtsam::SharedNoiseModel odomNoise_,
            ::gtsam::SharedNoiseModel cameraNoise_,
            std::map<gtsam::Key, gtsam::Cal3_S2> cameraCal_,
            std::vector<int> fixedTags_);

  /**
   * Optimize from a given [input] starting state
   */
  void Optimize(const OptimizerState &newThings);

private:
  gtsam::Key GetNearestStateToKeyframe(int64_t time);

  void AddOdometryFactors(const OptimizerState &newThings);

  void AddKeyframes(const OptimizerState &newThings);

  gtsam::ExpressionFactorGraph graph;
  gtsam::Values currentEstimate{};

  // initial tag layout guess. In the future, refactor to support totally
  // unstructured setups
  frc::AprilTagFieldLayout layoutGuess;

  gtsam::Pose3 wTb_latest;
  gtsam::Key latestRobotState;

  gtsam::SharedNoiseModel odomNoise;
  gtsam::SharedNoiseModel cameraNoise;

  std::map<gtsam::Key, gtsam::Cal3_S2> cameraCalMap;

  std::vector<int> fixedTags;

  // Keep track of timestamps/states from odometry
  TimeKeyMap timeToKeyMap;

  // If we've seen at -least- one keyframe
  bool gotAkeyframe = false;
};

} // namespace sfm_mapper
