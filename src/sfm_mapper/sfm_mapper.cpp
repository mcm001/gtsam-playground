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

#include "sfm_mapper.h"

#include "TagModel.h"
#include "gtsam_utils.h"
#include "helpers.h"

using namespace gtsam;
using namespace sfm_mapper;

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>

SfmMapper::SfmMapper(frc::AprilTagFieldLayout layoutGuess_,
                     ::gtsam::SharedNoiseModel odomNoise_,
                     ::gtsam::SharedNoiseModel cameraNoise_,
                     std::map<gtsam::Key, gtsam::Cal3_S2> cameraCal_)
    : layoutGuess{layoutGuess_}, odomNoise{odomNoise_},
      cameraNoise{cameraNoise_}, cameraCalMap(cameraCal_) {

  for (const auto &[key, cal] : cameraCalMap) {
    // todo - guess null
    currentEstimate.insert(helpers::CameraIdxToKey(key), Pose3{});
  }
}

void SfmMapper::AddOdometryFactors(ExpressionFactorGraph &graph,
                                   Values &initial,
                                   const OptimizerState &input) {
  if (!gotAkeyframe) {
    return;
  }

  for (const auto &odom : input.odometryMeasurements) {
    graph.emplace_shared<BetweenFactor<Pose3>>(odom.stateFrom, odom.stateTo,
                                               odom.poseDelta, odomNoise);

    wTb_latest = wTb_latest.transformPoseFrom(odom.poseDelta);
    initial.insert(odom.stateTo, wTb_latest);
    latestRobotState = odom.stateTo;
  }
}

void SfmMapper::AddKeyframes(ExpressionFactorGraph &graph, Values &initial,
                             const OptimizerState &input) {
  // This is the first time, so we need to add an initial guess for the start of
  // the pose backbone
  if (!gotAkeyframe) {
    if (input.keyframes.size()) {
      gotAkeyframe = true;

      auto tags = input.keyframes[0].observation;
      auto est = estimateWorldTcam(tags, layoutGuess);

      if (!est) {
        throw std::runtime_error("Couldn't find tag in map?");
        return;
      }

      wTb_latest = *est;
      initial.insert(latestRobotState, wTb_latest);
    } else {
      // give up, can't start
      return;
    }
  }

  for (const auto &keyframe : input.keyframes) {
    for (const TagDetection &tag : keyframe.observation) {
      auto tagKey{helpers::TagIdToKey(tag.id)};
      auto worldPcorners{TagModel::WorldToCornersFactor(tagKey)};

      // If ISAM doesn't yet know about this tag, we'll need to add an initial
      // guess
      if (!isam.valueExists(tagKey)) {
        auto tagOpt = helpers::GetTagPose(layoutGuess, tag.id);
        if (tagOpt) {
          // I'm too lazy to deal with seeing the same tag in two keyframes we
          // add at the same time, but GetTagPose will yield the same thing --
          // insert_or_assign is appropriate here
          initial.insert_or_assign(tagKey, Pose3dToGtsamPose3(tagOpt->pose));
        } else {
          throw std::runtime_error("Couldn't find tag guess?");
        }
      }

      constexpr int NUM_CORNERS = 4;
      for (size_t i = 0; i < NUM_CORNERS; i++) {
        // HACK: world->body is just attached to the newest robot state key
        const Pose3_ worldTbody_fac(latestRobotState);

        // Decision variable - where our camera is in the world
        const Pose3_ robotTcamera_fac(keyframe.cameraIdx);

        // Where we'd predict the i'th corner of the tag to be
        const auto prediction = PredictLandmarkImageLocationFactor(
            worldTbody_fac, robotTcamera_fac, cameraCalMap[keyframe.cameraIdx],
            worldPcorners[i]);

        // where we saw the i'th corner in the image
        Point2 measurement = {tag.corners[i].x, tag.corners[i].y};

        // Add this prediction/measurement pair to our graph
        graph.addExpressionFactor(prediction, measurement, cameraNoise);
      }
    }
  }
}

OptimizerState SfmMapper::Optimize(const OptimizerState &input) {
  ExpressionFactorGraph graph;

  AddOdometryFactors(graph, currentEstimate, input);
  AddKeyframes(graph, currentEstimate, input);

  isam.update(graph, currentEstimate);
  currentEstimate.clear();

  wTb_latest = isam.calculateEstimate<Pose3>(latestRobotState);

  OptimizerState ret;
  return ret;
}
