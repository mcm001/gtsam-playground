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
                     std::map<gtsam::Key, gtsam::Cal3_S2> cameraCal_,
                     std::vector<int> fixedTags_)
    : layoutGuess{layoutGuess_}, odomNoise{odomNoise_},
      cameraNoise{cameraNoise_}, cameraCalMap(cameraCal_),
      fixedTags{fixedTags_} {

  for (const auto &[key, cal] : cameraCalMap) {
    // todo - guess null
    currentEstimate.insert(helpers::CameraIdxToKey(key), Pose3{});
  }

  ExpressionFactorGraph graph;
  for (int tagId : fixedTags) {

    auto worldTtag1 = TagModel::worldToTag(tagId);
    if (!worldTtag1) {
      throw std::runtime_error("Couldnt find fixed tag in map!");
    }

    graph.emplace_shared<NonlinearEquality<Pose3>>(helpers::TagIdToKey(tagId),
                                                   *worldTtag1);
  }

  for (const frc::AprilTag &tag : layoutGuess.GetTags()) {
    currentEstimate.insert(helpers::TagIdToKey(tag.ID),
                           Pose3dToGtsamPose3(tag.pose));
  }

  graph.print("Initial factor list: ");
  currentEstimate.print("Initial guesses: ");
  isam.update(graph, currentEstimate);
  currentEstimate.clear();
}

using TimeKeyMap = SfmMapper::TimeKeyMap;
static TimeKeyMap::const_iterator FindCloser(TimeKeyMap::const_iterator left,
                                             TimeKeyMap::const_iterator right,
                                             int64_t time) {
  auto deltaLeft = time - left->first;
  auto deltaRight = right->first - time;
  if (deltaLeft < deltaRight) {
    return left;
  } else {
    return right;
  }
}

Key SfmMapper::GetNearestStateToKeyframe(int64_t time) {
  const auto &isamEntryAfter = timeToKeyMap.upper_bound(time);

  // Case 1: time might be before our map
  if (isamEntryAfter == timeToKeyMap.begin()) {
    if (time != timeToKeyMap.begin()->first) {
      throw std::runtime_error("Timestamp is before even isam history");
    } else {
      return timeToKeyMap.begin()->second;
    }
  }

  // case 2, time might be after
  if (isamEntryAfter == timeToKeyMap.end()) {
    throw std::runtime_error("Timestamp is after all keys in map?");
  }

  // safe to do this, we checked we aren't at the start
  const auto &isamEntryBefore = std::prev(isamEntryAfter);

  return FindCloser(isamEntryBefore, isamEntryAfter, time)->second;
}

void SfmMapper::AddOdometryFactors(ExpressionFactorGraph &graph,
                                   Values &initial,
                                   const OptimizerState &input) {
  if (!gotAkeyframe) {
    return;
  }

  for (const auto &odom : input.odometryMeasurements) {
    // if (odom.stateFrom != latestRobotState) {
    //   throw std::runtime_error("Is stuff out of order?");
    // }
    // if (odom.stateFrom - odom.stateTo != 1) {
    //   throw std::runtime_error("Why is to-from != 1?");
    // }

    graph.emplace_shared<BetweenFactor<Pose3>>(latestRobotState, latestRobotState + 1,
                                               odom.poseDelta, odomNoise);

    wTb_latest = wTb_latest.transformPoseFrom(odom.poseDelta);
    initial.insert(odom.stateTo, wTb_latest);

    timeToKeyMap[odom.time] = latestRobotState + 1;
    latestRobotState++;
  }
}

void SfmMapper::AddKeyframes(ExpressionFactorGraph &graph, Values &initial,
                             const OptimizerState &input) {
  // This is the first time, so we need to add an initial guess for the start of
  // the pose backbone
  if (!gotAkeyframe) {
    if (input.keyframes.size()) {
      gotAkeyframe = true;

      auto keyframe = input.keyframes[0];
      auto tags = keyframe.observation;
      gtsam::Cal3_S2 cal = cameraCalMap[keyframe.cameraIdx];
      auto est = estimateWorldTcam(tags, layoutGuess, cal.fx(), cal.fy(),
                                   cal.px(), cal.py());

      if (!est) {
        throw std::runtime_error("Couldn't find tag in map?");
        return;
      }

      wTb_latest = *est;
      initial.insert(latestRobotState, wTb_latest);
      timeToKeyMap[keyframe.time] = helpers::StateNumToKey(1);
    } else {
      // give up, can't start
      return;
    }
  }

  for (const auto &keyframe : input.keyframes) {
    for (const TagDetection &tag : keyframe.observation) {
      if (std::find(fixedTags.begin(), fixedTags.end(), tag.id) !=
          fixedTags.end()) {
        // tag is fixed -- todo figure out
      }

      auto tagKey{helpers::TagIdToKey(tag.id)};
      auto worldPcorners{TagModel::WorldToCornersFactor(tagKey)};

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

  // Add odom and shove into our graph so I don't have to deal with this weird partial thing
  AddOdometryFactors(graph, currentEstimate, input);
  graph.print("==================\nAdding odom factors: ");
  currentEstimate.print("with initial odom guess: ");
  isam.update(graph, currentEstimate);
  graph.clear();
  currentEstimate.clear();

  AddKeyframes(graph, currentEstimate, input);

  graph.print("Adding keyframe factors: ");
  currentEstimate.print("with initial keyframe guess: ");
  isam.update(graph, currentEstimate);
  graph.clear();
  currentEstimate.clear();

  wTb_latest = isam.calculateEstimate<Pose3>(latestRobotState);

  OptimizerState ret;
  return ret;
}
