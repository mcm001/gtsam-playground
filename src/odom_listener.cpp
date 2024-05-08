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

#include "odom_listener.h"

#include <networktables/NetworkTableInstance.h>

#include "gtsam_utils.h"

using std::vector;
using namespace gtsam;

static Vector6 makeOdomNoise(const LocalizerConfig &config) {
  return Vector6{config.rotNoise[0],   config.rotNoise[1],
                 config.rotNoise[2],   config.transNoise[0],
                 config.transNoise[1], config.transNoise[2]};
}

OdomListener::OdomListener(LocalizerConfig config)
    : odomSub(nt::NetworkTableInstance::GetDefault()
                  .GetStructTopic<frc::Twist3d>(config.rootTableName +
                                                "/input/odom_twist")
                  .Subscribe({},
                             {
                                 .pollStorage = 100,
                                 .sendAll = true,
                                 .keepDuplicates = true,
                             })),
      initialGuessSub(
          nt::NetworkTableInstance::GetDefault()
              .GetStructTopic<frc::Pose3d>(config.rootTableName +
                                           "/input/pose_initial_guess")
              .Subscribe({},
                         {
                             .pollStorage = 100,
                             .sendAll = true,
                             .keepDuplicates = true,
                         })),
      odomNoise(noiseModel::Diagonal::Sigmas(
          // Odoometry factor stdev: rad,rad,rad,m, m, m
          makeOdomNoise(config))),
      priorNoise(noiseModel::Diagonal::Sigmas(
          // initial guess stdev: rad,rad,rad,m, m, m
          (Vector(6) << 1, 1, 1, 1, 1, 1).finished())) {}

std::optional<Timestamped<Pose3WithNoise>> OdomListener::NewPosePrior() {
  auto ret = newPriorPose;
  newPriorPose = std::nullopt;
  return ret;
}

bool OdomListener::ReadyToOptimize() {
  if (!hasInitialGuess) {
    // grab the latest robot-cam transform
    const auto last_wTr = initialGuessSub.GetAtomic();
    // if not published, time will be zero
    if (last_wTr.time == 0) {
      fmt::println("Global: no wTr guess set?");
      return false;
    }

    fmt::println("Global: Resetting localizer at {}", last_wTr.time);

    Pose3 wTr = Pose3dToGtsamPose3(last_wTr.value);
    newPriorPose = {last_wTr.time, {wTr, priorNoise}};

    hasInitialGuess = true;
  }

  return hasInitialGuess;
}

std::vector<OdometryObservation> OdomListener::Update() {
  const auto odom = odomSub.ReadQueue();

  std::vector<OdometryObservation> ret;
  ret.reserve(odom.size());

  for (const auto &o : odom) {
    auto &twist = o.value;

    Vector6 eigenTwist;
    eigenTwist << twist.rx.to<double>(), twist.ry.to<double>(),
        twist.rz.to<double>(), twist.dx.to<double>(), twist.dy.to<double>(),
        twist.dz.to<double>();
    Pose3 odomPoseDelta = Pose3::Expmap(eigenTwist);

    ret.emplace_back(o.time, odomPoseDelta, odomNoise);
  }

  return ret;
}
