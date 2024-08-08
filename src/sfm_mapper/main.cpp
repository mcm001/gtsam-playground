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

#include <fmt/core.h>
#include <ntcore_cpp_types.h>

#include <variant>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <wpi/DataLogReader.h>
#include <wpi/DenseMap.h>

#include "TagModel.h"
#include "gtsam_utils.h"

#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include "TagDetection.h"
#include <vector>

#include <frc/geometry/Twist3d.h>
#include <gtsam/inference/Symbol.h>
#include "PhotonPoseEstimator.h"

#include <fstream>
#include <sstream>
#include <string>
#include <wpi/json.h>

#include "helpers.h"

using namespace std;
using namespace gtsam;
using namespace noiseModel;
using symbol_shorthand::L;
using symbol_shorthand::X;

static frc::AprilTagFieldLayout tagLayoutGuess =
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);


int main() {
  // Camera calibration parameters. Order is [fx fy skew cx cy] in pixels
  Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
  Isotropic::shared_ptr cameraNoise =
      Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Noise on our pose prior. order is rx, ry, rz, tx, ty, tz, and units are
  // [rad] and [m]
  Vector6 sigmas;
  sigmas << Vector3::Constant(0.1), Vector3::Constant(0.3);
  auto posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

  // Our platform and camera are coincident
  gtsam::Pose3 robotTcamera{};

  // constant Expression for the calibration we can reuse
  Cal3_S2_ cameraCal(K);

  // ======================

  // Map of [observation state ID] to [tags seen]
  map<Key, vector<TagDetection>> points = ParseFile();

  // Create a factor graph
  ExpressionFactorGraph graph;

  // Add all our tag observations
  for (const auto &[stateKey, tags] : points) {
    for (const TagDetection &tag : tags) {
      auto worldPcorners = TagModel::WorldToCornersFactor(L(tag.id));

      // add each tag corner
      constexpr int NUM_CORNERS = 4;
      for (size_t i = 0; i < NUM_CORNERS; i++) {
        // Decision variable - where our camera is in the world
        const Pose3_ worldTbody_fac(stateKey);
        // Where we'd predict the i'th corner of the tag to be
        const auto prediction = PredictLandmarkImageLocationFactor(
            worldTbody_fac, robotTcamera, cameraCal, worldPcorners[i]);
        // where we saw the i'th corner in the image
        Point2 measurement = {tag.corners[i].x, tag.corners[i].y};
        // Add this prediction/measurement pair to our graph
        graph.addExpressionFactor(prediction, measurement, cameraNoise);
      }
    }
  }

  // Add pose prior on our fixed tag. Right now, foce it to be a tag in our dataset
  int FIXED_TAG = 10;
  auto worldTtag1 = TagModel::worldToTag(FIXED_TAG);
  if (!worldTtag1) {
    fmt::println("Couldnt find tag {} in map!", FIXED_TAG);
  }
  graph.addPrior(L(FIXED_TAG), *worldTtag1, posePriorNoise);

  // Initial guess for our optimizer. Needs to be in the right ballpark, but
  // accuracy doesn't super matter
  Values initial;

  // Guess for all camera poses
  for (const auto &[stateKey, tags] : points) {
    // Initial guess at camera pose based on regular old multi tag pose
    // esitmation
    auto worldTcam_guess = estimateObservationPose(tags, tagLayoutGuess);
    if (!worldTcam_guess) {
      std::cerr << "Can't guess pose of camera for observation " << stateKey << std::endl;;
    } else {
      initial.insert<Pose3>(stateKey, *worldTcam_guess);
    }
  }

  // Guess for tag locations
  for (const frc::AprilTag &tag : tagLayoutGuess.GetTags()) {
    initial.insert(L(tag.ID), Pose3dToGtsamPose3(tag.pose));
  }

  graph.print("Final pose graph\n");

  /* Optimize the graph and print results */
  cout << "initial error = " << graph.error(initial) << endl;
  initial.print("Initial state:\n");
  Values result = DoglegOptimizer(graph, initial).optimize();
  cout << "final error = " << graph.error(result) << endl;
  result.print("Final state:\n");

  return 0;
}
