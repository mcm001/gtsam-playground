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

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include <fstream>

#include "TagModel.h"
#include "gtsam_utils.h"
#include "helpers.h"

using namespace gtsam;
using namespace sfm_mapper;

// Pose3 tangent representation is [ Rx Ry Rz Tx Ty Tz ].
static const int kIndexRx = 0;
static const int kIndexRy = 1;
static const int kIndexRz = 2;
static const int kIndexTx = 3;
static const int kIndexTy = 4;
static const int kIndexTz = 5;

// How sure we are about "fixed" tags, order is [rad rad rad m m m]
noiseModel::Diagonal::shared_ptr posePriorNoise = noiseModel::Diagonal::Sigmas(
    (Vector(6, 1) << 0.015, 0.015, 0.015, 0.005, 0.005, 0.005).finished());

SfmMapper::SfmMapper(frc::AprilTagFieldLayout layoutGuess_,
                     ::gtsam::SharedNoiseModel odomNoise_,
                     ::gtsam::SharedNoiseModel cameraNoise_,
                     std::map<gtsam::Key, gtsam::Cal3_S2> cameraCal_,
                     std::vector<int> fixedTags_)
    : layoutGuess{layoutGuess_}, odomNoise{odomNoise_},
      cameraNoise{cameraNoise_}, cameraCalMap(cameraCal_),
      fixedTags{fixedTags_} {

  ExpressionFactorGraph graph;

  for (const auto &[key, cal] : cameraCalMap) {
    // todo - guess null
    currentEstimate.insert(helpers::CameraIdxToKey(key),
                           Pose3{Rot3::Ry(-.3), Point3{}});

    // // hack - constrain robot->cam
    // graph.emplace_shared<PriorFactor<Pose3>>(helpers::CameraIdxToKey(key),
    //                                          Pose3{Rot3::Ry(-.3), Point3{}},
    //                                          posePriorNoise);
  }

  for (int tagId : fixedTags) {

    auto worldTtag = TagModel::worldToTag(tagId);
    if (!worldTtag) {
      throw std::runtime_error("Couldnt find fixed tag in map!");
    }

    graph.emplace_shared<NonlinearEquality<Pose3>>(helpers::TagIdToKey(tagId),
                                                   *worldTtag);
    // graph.emplace_shared<PriorFactor<Pose3>>(helpers::TagIdToKey(tagId),
    //                                          *worldTtag, posePriorNoise);
  }

  for (const frc::AprilTag &tag : layoutGuess.GetTags()) {
    currentEstimate.insert(helpers::TagIdToKey(tag.ID),
                           Pose3dToGtsamPose3(tag.pose));
  }

  graph.print("Initial factor list: ");
  currentEstimate.print("Initial guesses: ");
  isam.update(graph, currentEstimate);
  currentEstimate.clear();

  graph.saveGraph("graph_ctor.dot", currentEstimate);
}

static void ConstrainToFloor(ExpressionFactorGraph &graph, gtsam::Key key) {
  auto logmap = Rot3::Logmap(Rot3::Identity());
  auto vec = gtsam::Vector(4);
  vec[0] = 0;
  vec.block(1, 0, 3, 1) = logmap;
  std::cout << "logmap: " << logmap << std::endl;
  std::cout << "Constraining to: " << vec << std::endl;
  std::vector<size_t> indices{
      kIndexTz,
      kIndexRx,
      kIndexRy,
      kIndexRz,
  };
  graph.emplace_shared<PartialPriorFactor<Pose3>>(
      key, indices, vec, noiseModel::Isotropic::Sigma(4, 0.01));
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
    graph.emplace_shared<BetweenFactor<Pose3>>(
        latestRobotState, latestRobotState + 1, odom.poseDelta, odomNoise);

    wTb_latest = wTb_latest.transformPoseFrom(odom.poseDelta);
    initial.insert(latestRobotState + 1, wTb_latest);

    timeToKeyMap[odom.time] = latestRobotState + 1;
    latestRobotState++;

    ConstrainToFloor(graph, latestRobotState);
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

      latestRobotState = helpers::StateNumToKey(1);

      initial.insert(latestRobotState, wTb_latest);
      timeToKeyMap[keyframe.time] = latestRobotState;

      ConstrainToFloor(graph, latestRobotState);
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

  // Add odom and shove into our graph so I don't have to deal with this weird
  // partial thing
  AddOdometryFactors(graph, currentEstimate, input);
  graph.print("==================\nAdding odom factors: ");
  currentEstimate.print("with initial odom guess: ");
  isam.update(graph, currentEstimate);
  graph.resize(0);
  currentEstimate.clear();

  AddKeyframes(graph, currentEstimate, input);

  graph.print("Adding keyframe factors: ");
  currentEstimate.print("with initial keyframe guess: ");
  graph.saveGraph("graph_keyframes.dot", currentEstimate);
  isam.update(graph, currentEstimate);
  graph.resize(0);
  currentEstimate.clear();

  wTb_latest = isam.calculateEstimate<Pose3>(latestRobotState);

  OptimizerState ret;
  return ret;
}
