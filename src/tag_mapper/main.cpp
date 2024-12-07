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

#include "mapper_nt_iface.h"
#include "wpical/wpical.h"
#include <gtsam/linear/NoiseModel.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

using namespace gtsam;
using namespace std::chrono_literals;
using std::cout;
using std::endl;

int main() {
  wpical::GtsamApriltagMap tagMap {
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2024Crescendo),
    6.5_in
  };

  // one pixel in u and v
  SharedNoiseModel cameraNoise{
      noiseModel::Isotropic::Sigma(2, 1.0)}; 

  // Noise on our pose prior. order is rx, ry, rz, tx, ty, tz, and units are
  // [rad] and [m].
  // Guess ~1 degree and 5 mm for fun.
  Vector6 sigmas;
  sigmas << Vector3::Constant(0.015), Vector3::Constant(0.005);
  SharedNoiseModel posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

  const int32_t FIXED_TAG = 7;
  const std::map<int32_t, std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>> fixedTags {
    {FIXED_TAG, { tagMap.WorldToTag(FIXED_TAG).value(), posePriorNoise } }
  };

  // Camera calibration parameters. Order is [fx fy skew cx cy] in pixels. Fixed for now, since I'm lazy 
  const double cam_fx = 5.9938E+02;
  const double cam_fy = 5.9917E+02;
  const double cam_cx = 4.7950E+02;
  const double cam_cy = 3.5950E+02;
  Cal3_S2 cal{cam_fx, cam_fy, 0.0, cam_cx, cam_cy};

  wpical::KeyframeMap keyframes;
  MapperNtIface ntIface;

  while (true) {
    // rate limit loop

    std::this_thread::sleep_for(1000ms);

    // Map of [observation state ID] to [tags seen]
    wpical::KeyframeMap newObservations = ntIface.NewKeyframes();

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

    auto result { OptimizeLayout(tagMap, keyframes, cal, fixedTags, cameraNoise) };

    std::vector<Pose3WithVariance> tags;
    std::vector<Pose3WithVariance> camera;

    for (const auto& [key, val] : result.tagPoseCovariances) {
      tags.push_back(val);
    }
    for (const auto& [key, val] : result.cameraPoseCovariances) {
      camera.push_back(val);
    }

    ntIface.PublishResult(result.optimizedLayout, tags, camera);
  }

  return 0;
}
