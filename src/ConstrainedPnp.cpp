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

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include <iostream>
#include <vector>

#include "TagDetection.h"
#include "TagModel.h"
#include "gtsam_utils.h"

using namespace gtsam;
using symbol_shorthand::X;

// Pose3 tangent representation is [ Rx Ry Rz Tx Ty Tz ].
static const int kIndexRx = 0;
static const int kIndexRy = 1;
static const int kIndexRz = 2;
static const int kIndexTx = 3;
static const int kIndexTy = 4;
static const int kIndexTz = 5;

/*
transform to change from the wpilib/photon default (camera optical axis
along +x) to the standard from opencv (z along optical axis)

We want:
x in [0,-1,0]
y in [0,0,-1]
z in [1,0,0]
*/
const Pose3 wpilibToCvCameraTransform{
    Pose3{Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), Point3{0.0, 0, 0.0}}};

// Noise model to use on our pose-partial-prior
// TODO - expose this as something tunable
::gtsam::SharedNoiseModel partialPosePriorNoise = noiseModel::Diagonal::Sigmas(
    (Vector(4, 1) << 0.001, 0.01, 0.01, 0.05).finished());

/**
 * Constrain my robot to be flat on the 2d floor plane at Z=0. This means we
 * want the Z coordinate to be 0, and the rotation about X and Y to be 0.
 *
 * Further, we constrain the yaw, leaving only Tx and Ty fully unconstrained.
 *
 * Note that the local tangent space for SE(3) is encoded in order (rx ry rz tx
 * ty tz), per Pose3.h's interval documentation.
 */
static void AddPosePriorFloorGyro(ExpressionFactorGraph &graph, gtsam::Key key,
                                  double yawEst) {
  auto logmap = Rot3::Logmap(Rot3::Rz(yawEst));

  auto vec = gtsam::Vector(4);
  vec[0] = 0;         // Tz
  vec[1] = logmap(0); // Rx
  vec[2] = logmap(1); // Ry
  vec[3] = logmap(2); // Rz

  const std::vector<size_t> indices{
      kIndexTz,
      kIndexRx,
      kIndexRy,
      kIndexRz,
  };

  graph.emplace_shared<PartialPriorFactor<Pose3>>(key, indices, vec,
                                                  partialPosePriorNoise);
}

const Key robotPose{X(0)};
const Pose3_ worldTbody_fac{robotPose};

struct PoseEstimator {
  ExpressionFactorGraph graph{};
  Values currentEstimate;
  gtsam::SharedNoiseModel cameraNoise{noiseModel::Isotropic::Sigma(2, 1.0)};
  gtsam::Pose3 robotTcamera_wpilib;
  gtsam::Cal3_S2 cameraCal;

  /**
   * Clear all factors and reset our estimate
   */
  void Reset() {
    graph.resize(0);
    currentEstimate.clear();
  }

  /**
   * Set where we think the robot is in the field. The yaw component of this
   * pose is used as a prior on robot yaw
   */
  void SetRobotPoseGuess(Pose3 pose) {
    currentEstimate.insert_or_assign(robotPose, pose);

    AddPosePriorFloorGyro(graph, robotPose, pose.rotation().yaw());
  }

  /**
   * Add a tag observation set from the current timestep
   */
  void AddKeyframes(std::vector<TagDetection> observation) {
    for (const TagDetection &tag : observation) {
      auto worldPcorners{TagModel::WorldToCorners(tag.id)};
      if (!worldPcorners) {
        fmt::println("Unknown tag {}", tag.id);
        continue;
      }

      constexpr int NUM_CORNERS = 4;
      for (size_t i = 0; i < NUM_CORNERS; i++) {
        const Pose3 robotTcamera_cv{robotTcamera_wpilib *
                                    wpilibToCvCameraTransform};

        // Where we'd predict the i'th corner of the tag to be
        const Point2_ prediction{PredictLandmarkImageLocation(
            worldTbody_fac, robotTcamera_cv, cameraCal, (*worldPcorners)[i])};

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

  /**
   * Solve the optimization problem for the robot pose
   */
  const Pose3 Solve() const {
    DoglegParams params;
    params.verbosity = NonlinearOptimizerParams::ERROR;

    DoglegOptimizer optimizer{graph, currentEstimate, params};

    auto values = optimizer.optimize();

    values.at<Pose3>(robotPose).print("Estimated pose");

    return values.at<Pose3>(robotPose);
  }
};

int main() {
  PoseEstimator estimator{};
  estimator.robotTcamera_wpilib = Pose3{};
  estimator.cameraCal = {1000, 1000, 0, 320, 240};

  estimator.Reset();
  estimator.SetRobotPoseGuess(Pose3{});
  estimator.AddKeyframes({});

  estimator.Solve();

  return 0;
}
