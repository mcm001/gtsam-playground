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

#include <fmt/core.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
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

/*
add transform to change from the wpilib/photon default (camera optical axis
along +x) to the standard from opencv (z along optical axis)

We want:
x in [0,-1,0]
y in [0,0,-1]
z in [1,0,0]
*/
const gtsam::Pose3_ wpilibToCvCameraTransform{gtsam::Pose3{
    gtsam::Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), gtsam::Point3{0.0, 0, 0.0}}};

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

  for (const auto &[key, cal] : cameraCalMap) {
    // todo - guess null
    Pose3 camGuess{Rot3::Ry(-.15), Point3{.4, 0, .4}};

    currentEstimate.insert(helpers::CameraIdxToKey(key), camGuess);
  }

  for (int tagId : fixedTags) {

    auto worldTtag = TagModel::worldToTag(tagId);
    if (!worldTtag) {
      throw std::runtime_error("Couldnt find fixed tag in map!");
    }

    graph.emplace_shared<NonlinearEquality<Pose3>>(helpers::TagIdToKey(tagId),
                                                   *worldTtag);
  }

  for (const frc::AprilTag &tag : layoutGuess.GetTags()) {
    currentEstimate.insert(helpers::TagIdToKey(tag.ID),
                           Pose3dToGtsamPose3(tag.pose));
  }

  graph.print("Initial factor list: ");
  currentEstimate.print("Initial guesses: ");
  graph.saveGraph("graph_ctor.dot", currentEstimate);
}

/**
 * Constrain my robot to be flat on the 2d floor plane at Z=0. This means we
 * want the Z coordinate to be 0, and the rotation about X and Y to be 0.
 *
 * Note that the local tangent space for SE(3) encoded as (rx ry rz tx ty tz).
 */
static void ConstrainToFloor(ExpressionFactorGraph &graph, gtsam::Key key) {
  auto logmap = Rot3::Logmap(Rot3::Identity());
  auto vec = gtsam::Vector(3);

  vec[0] = 0;         // Tz
  vec[0] = logmap(0); // Rx
  vec[0] = logmap(1); // Ry

  std::vector<size_t> indices{
      kIndexTz,
      kIndexRx,
      kIndexRy,
  };

  graph.emplace_shared<PartialPriorFactor<Pose3>>(
      key, indices, vec, noiseModel::Isotropic::Sigma(3, 0.1));
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
      throw std::runtime_error("Timestamp is before our history");
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

void SfmMapper::AddOdometryFactors(const OptimizerState &input) {
  for (const auto &odom : input.odometryMeasurements) {
    graph.emplace_shared<BetweenFactor<Pose3>>(
        latestRobotState, latestRobotState + 1, odom.poseDelta, odomNoise);

    Symbol(latestRobotState).print("state was ");
    wTb_latest.print(" w2b was");
    odom.poseDelta.print("pose delta");
    wTb_latest = wTb_latest.transformPoseFrom(odom.poseDelta);
    Symbol(latestRobotState).print("state will be ");
    wTb_latest.print(" w2b now");

    currentEstimate.insert(latestRobotState + 1, wTb_latest);

    timeToKeyMap[odom.time] = latestRobotState + 1;
    latestRobotState++;

    ConstrainToFloor(graph, latestRobotState);
  }
}

void SfmMapper::AddKeyframes(const OptimizerState &input) {
  for (const auto &keyframe : input.keyframes) {
    fmt::println("Adding keyframe at time {}", keyframe.time);

    for (const TagDetection &tag : keyframe.observation) {
      auto tagKey{helpers::TagIdToKey(tag.id)};
      auto worldPcorners{TagModel::WorldToCornersFactor(tagKey)};
      const Pose3_ worldTbody_fac{GetNearestStateToKeyframe(keyframe.time)};

      constexpr int NUM_CORNERS = 4;
      for (size_t i = 0; i < NUM_CORNERS; i++) {

        // Decision variable - where our camera is in the world
        const Pose3_ robotTcamera_wpilib{keyframe.cameraIdx};
        const Pose3_ robotTcamera_cv{robotTcamera_wpilib *
                                     wpilibToCvCameraTransform};

        // Where we'd predict the i'th corner of the tag to be
        const Point2_ prediction = PredictLandmarkImageLocationFactor(
            worldTbody_fac, robotTcamera_cv, cameraCalMap[keyframe.cameraIdx],
            worldPcorners[i]);

        // where we saw the i'th corner in the image
        Point2 measurement = {tag.corners[i].x, tag.corners[i].y};

        // Add this prediction/measurement pair to our graph
        graph.addExpressionFactor(prediction, measurement, cameraNoise);

        Point2 initialError = measurement - prediction.value(currentEstimate);
        fmt::println("tag {} initial error ({}, {})/L2 norm {:.2f}px", tag.id,
                     initialError(0), initialError(1), initialError.norm());
      }
    }
  }
}

void SfmMapper::Optimize(const OptimizerState &input) {
  // need initial guess for odom FK bullshit. This guess does need to be
  // reasonably good
  // TODO: telemeter this from the robot maybe?
  if (!gotAkeyframe) {
    gotAkeyframe = true;

    auto keyframe = input.keyframes[0];

    // hack - take pose from the first tagdetection
    wTb_latest =
        Pose3dToGtsamPose3(frc::Pose3d{keyframe.observation[0].poseGuess});

    latestRobotState = helpers::StateNumToKey(1);

    currentEstimate.insert(latestRobotState, wTb_latest);
    timeToKeyMap[keyframe.time] = latestRobotState;

    ConstrainToFloor(graph, latestRobotState);
  }

  AddOdometryFactors(input);
  graph.print("==================\nAdding odom factors: ");
  currentEstimate.print("with initial odom guess: ");

  fmt::println("And times:");
  for (const auto &[time, key] : timeToKeyMap) {
    std::cout << time << " -> " << Symbol(key) << std::endl;
    ;
  }

  AddKeyframes(input);
  graph.print("Adding keyframe factors: ");
  currentEstimate.print("with initial keyframe guess: ");

  graph.saveGraph("graph_keyframes.dot", currentEstimate);

  auto initialErr = graph.error(currentEstimate);
  fmt::println("Initial error: {}", initialErr);

  // Do the thing

  // DoglegParams params;
  // params.verbosity = NonlinearOptimizerParams::ERROR;
  // DoglegOptimizer optimizer{graph, currentEstimate, params};
  // currentEstimate = optimizer.optimize();

  ISAM2Params parameters;
  // parameters.relinearizeThreshold = 0.01;
  // parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  parameters.print();
  ISAM2 isam(parameters);
  isam.update(graph, currentEstimate);
  currentEstimate = isam.calculateEstimate();

  wTb_latest = currentEstimate.at<Pose3>(latestRobotState);

  auto finalErr = graph.error(currentEstimate);
  fmt::println("Final error: {} (a reduction of {}x)", finalErr,
               initialErr / finalErr);

  // end doing the thing
}
