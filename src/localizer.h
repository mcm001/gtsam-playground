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
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <map>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <units/time.h>

#include "gtsam/slam/expressions.h"
#include "gtsam_utils.h"

class Localizer {
  using Key = gtsam::Key;
  using SmartFactor = gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>;
  using LandmarkMap = std::map<Key, SmartFactor::shared_ptr>;

public:
  Localizer();

  /**
   * Add a prior factor on the world->robot pose
   */
  void Reset(gtsam::Pose3 wTr, gtsam::SharedNoiseModel noise, uint64_t timeUs);

  void AddOdometry(OdometryObservation odom);

  void AddTagObservation(CameraVisionObservation tagDetection);

  void Optimize();

  inline void Print(const std::string_view prefix = "") {
    fmt::println("{}", prefix);
    smootherISAM2.print();
    smootherISAM2.getFactorsUnsafe().print();
    smootherISAM2.calculateEstimate().print("Current estimate:");
  }

  inline Key GetCurrStateIdx() const { return currStateIdx; }
  inline uint64_t GetLastOdomTime() const { return latestOdomTime; }

  inline const gtsam::Pose3 GetLatestWorldToBody() const { return wTb_latest; }

  gtsam::Matrix GetLatestMarginals() const;
  // standard deviations on rx ry rz tx ty tz
  gtsam::Vector6 GetPoseComponentStdDevs() const;

  const std::vector<frc::Pose3d> GetPoseHistory() const;

protected:
  /**
   * If a given time is fully within the smoother history, find or interplate a
   * key for it
   */
  Key InsertIntoSmoother(Key lower, Key upper, Key newKey, double newTime,
                         gtsam::SharedNoiseModel odometryNoise);

  Key GetOrInsertKey(Key newKey, double time);

  // New factor graph to add to our smoother at the next call to Optimize()
  gtsam::ExpressionFactorGraph graph{};
  // New inital guesses to add to our smoother at the next call to Optimize()
  gtsam::Values currentEstimate{};
  // New state timestamps to add to our smoother at the next call to Optimize()
  gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps{};
  // Factors to delete
  gtsam::FactorIndices factorsToRemove{};

  // Log of old twists
  // typedef std::map<Key, gtsam::Pose3> KeyPoseDeltaMap;
  // KeyPoseDeltaMap twistsFromPreviousKey{};

  // ISAM + map of keys <-> times
  gtsam::ISAM2 smootherISAM2 {};
  std::map<gtsam::Key, double> keyToTimestamp {};

  // Current "tip" world->body estimate
  gtsam::Pose3 wTb_latest;
  uint64_t latestOdomTime;

  // keep track of our current state. State is encoded as X(uS since epoch).
  // the Key class uses the lower 56 bits for the index, and top 8 for symbol
  // 2^(64−8)÷10^6÷60÷60÷24÷365 = 2284 years, so as long as we use a sane epoch
  // we're good. This will only work on 64-bit machines, but oh well. big shame.
  Key currStateIdx;
};
