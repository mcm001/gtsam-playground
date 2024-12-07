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
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>
#include <ntcore_cpp_types.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Twist3d.h>
#include <wpi/DataLogReader.h>
#include <wpi/DenseMap.h>
#include <wpi/json.h>

#include "PhotonPoseEstimator.h"
#include "TagDetection.h"
#include "TagModel.h"
#include "gtsam_utils.h"
#include "helpers.h"
#include "mapper_nt_iface.h"

using std::cout;
using std::endl;
using std::map;
using std::vector;
using namespace gtsam;
using namespace noiseModel;
using symbol_shorthand::L;
using symbol_shorthand::X;
using namespace std::chrono_literals;

static frc::AprilTagFieldLayout tagLayoutGuess =
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

using KeyframeMap = std::map<gtsam::Key, std::vector<TagDetection>>;

class TagMapper {
private:
  MapperNtIface ntIface;

  // Camera calibration parameters. Order is [fx fy skew cx cy] in pixels
  Cal3_S2 K{cam_fx, cam_fy, 0.0, cam_cx, cam_cy};
  // constant Expression for the calibration we can reuse
  Cal3_S2_ cameraCal{K};

  Isotropic::shared_ptr cameraNoise{
      Isotropic::Sigma(2, 1.0)}; // one pixel in u and v

  noiseModel::Diagonal::shared_ptr posePriorNoise;

  // Our platform and camera are coincident
  gtsam::Pose3 robotTcamera{};

  void AddPosePrior(ExpressionFactorGraph &graph,
                    const KeyframeMap &keyframes) {
    const int FIXED_TAG = 7;
    auto worldTtag1 = TagModel::worldToTag(FIXED_TAG);
    if (!worldTtag1) {
      fmt::println("Couldnt find fixed tag {} in map!", FIXED_TAG);
    }
    graph.addPrior(L(FIXED_TAG), *worldTtag1, posePriorNoise);
  }

public:
  TagMapper() {
    // Noise on our pose prior. order is rx, ry, rz, tx, ty, tz, and units are
    // [rad] and [m].
    // Guess ~1 degree and 5 mm for fun.
    Vector6 sigmas;
    sigmas << Vector3::Constant(0.015), Vector3::Constant(0.005);
    posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);
  }

  inline MapperNtIface &NtIface() { return ntIface; }

  void OptimizeLayout(const KeyframeMap &keyframes) {
    // Create a factor graph to save -all- factors. Never cleared. Fast enough i
    // dont need isam here
    ExpressionFactorGraph graph;

    // constrain root tag
    AddPosePrior(graph, keyframes);

    // Add all our tag observation factors
    for (const auto &[stateKey, tags] : keyframes) {
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

    // Initial guess for our optimizer. Needs to be in the right ballpark, but
    // accuracy doesn't super matter
    Values initial;

    // Guess for all camera poses based on tag layout JSON
    for (const auto &[stateKey, tags] : keyframes) {
      auto worldTcam_guess = estimateWorldTcam(tags, tagLayoutGuess);
      if (!worldTcam_guess) {
        std::cerr << "Can't guess pose of camera for observation " << stateKey
                  << std::endl;
      } else {
        initial.insert<Pose3>(stateKey, *worldTcam_guess);
      }
    }

    // Guess for tag locations = tag layout json
    for (const frc::AprilTag &tag : tagLayoutGuess.GetTags()) {
      if (tagWasUsed(keyframes, tag.ID)) {
        initial.insert(L(tag.ID), Pose3dToGtsamPose3(tag.pose));
      }
    }

    /* Optimize the graph and print results */
    cout << "==========================\ninitial error = "
         << graph.error(initial) << endl;
    auto start = std::chrono::steady_clock::now();

    DoglegParams params;
    params.verbosity = NonlinearOptimizerParams::ERROR;
    // params.relativeErrorTol = 1e-3;
    // params.absoluteErrorTol = 1e-3;

    // Create initial optimizer
    DoglegOptimizer optimizer{graph, initial, params};

    // Run full optimization until convergence.
    Values result;
    try {
      result = optimizer.optimize();
    } catch (std::exception *e) {
      std::cerr << e->what();
      return;
    }

    auto end = std::chrono::steady_clock::now();
    auto dt = end - start;
    long long microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(dt).count();

    cout << "\n===== Converged in " << optimizer.iterations() << " iterations ("
         << microseconds << " uS) with final error " << optimizer.error()
         << " ======" << endl;

    cout << "Results:" << endl;

    std::stringstream ss;
    ss << "tag_map_" << result.size() << ".dot";
    graph.saveGraph(ss.str(), result);

    {
      gtsam::Marginals marginals(graph, result);
      std::vector<frc::AprilTag> tags;

      for (auto [key, value] : result) {
        std::cout << "\n========= Key " << gtsam::Symbol(key)
                  << " ==========\n";

        // Assume all our keys are pose3 factors. lol.
        auto est = GtsamToFrcPose3d(result.at<gtsam::Pose3>(key));
        fmt::println("Estimated pose:");
        fmt::println("Translation: x={:.2f} y={:.2f} z={:.2f}", est.X(),
                     est.Y(), est.Z());
        fmt::println("Rotation: W={:.3f} X={:.3f} Y={:.3f} Z={:.3f}",
                     est.Rotation().GetQuaternion().W(),
                     est.Rotation().GetQuaternion().X(),
                     est.Rotation().GetQuaternion().Y(),
                     est.Rotation().GetQuaternion().Z());

        // Covariance is the variance of x_i with x_i - stddev is sqrt(var)
        std::cout << "Marginal covariance (r t):" << endl
                  << marginals.marginalCovariance(key).diagonal().cwiseSqrt()
                  << std::endl;

        // todo - track all tag keys instead of this hack
        if (key >= L(0) && key <= L(2000)) {
          tags.push_back(frc::AprilTag{key - L(0), est});
        }
      }

      frc::AprilTagFieldLayout layout{tags, 16.541_m, 8.211_m};
      ntIface.PublishLayout(layout);
    }
  }
};

int main() {

  TagMapper mapper{};
  KeyframeMap keyframes{};

  while (true) {
    // rate limit loop
    std::this_thread::sleep_for(1000ms);

    // Map of [observation state ID] to [tags seen]
    KeyframeMap newObservations = mapper.NtIface().NewKeyframes();

    cout << "Got " << newObservations.size() << " things:" << endl;
    for (const auto [key, tagDets] : newObservations) {
      cout << gtsam::Symbol(key) << "(tags ";
      for (const auto tag : tagDets) {
        cout << tag.id << " ";
      }
      cout << ")" << endl;
    }

    if (!newObservations.size()) {
      cout << "no new keyframes - waiting\n";
      continue;
    }

    // Add keys not yet in keyframes from newObservations. We should never have
    // snapshot index colissions anyways.
    keyframes.merge(newObservations);

    mapper.OptimizeLayout(keyframes);
  }

  return 0;
}
