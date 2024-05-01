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

#include <frc/geometry/Pose3d.h>
#include <units/time.h>

#include "gtsam/slam/expressions.h"

using namespace gtsam;
using std::map;
using std::vector;
using symbol_shorthand::X;

class Localizer {
  using SmartFactor = SmartProjectionPoseFactor<Cal3_S2>;
  using LandmarkMap = map<Key, SmartFactor::shared_ptr>;

public:
  Localizer();

  /**
   * Add a prior factor on the world->robot pose
   */
  void Reset(Pose3 wTr, SharedNoiseModel noise, uint64_t timeUs);

  void AddOdometry(Pose3 twist, SharedNoiseModel noise, uint64_t timeUs);

  void AddTagObservation(int tagID, Cal3_S2_ cameraCal, Pose3 robotTcamera,
                         vector<Point2> corners, SharedNoiseModel noise,
                         uint64_t timeUs);

  void Optimize();

  // inline void ExportGraph(std::ostream& os) {
  //   smootherISAM2.getFactors().saveGraph(os);
  // }
  inline void Print(const std::string_view prefix = "") {
    fmt::println("{}", prefix);
    smootherISAM2.print();
    smootherISAM2.getISAM2().getFactorsUnsafe().print();
    smootherISAM2.calculateEstimate().print("Current estimate:");
  }

  inline Key GetCurrStateIdx() const { return currStateIdx; }

  inline const Pose3 GetLatestWorldToBody() const { return wTb_latest; }

  Matrix GetLatestMarginals() const;
  // standard deviations on rx ry rz tx ty tz
  Vector6 GetPoseComponentStdDevs() const;

  const vector<frc::Pose3d> GetPoseHistory() const;

protected:
  /**
   * If a given time is fully within the smoother history, find or interplate a
   * key for it
   */
  Key InsertIntoSmoother(Key lower, Key upper, Key newKey, double newTime,
                         SharedNoiseModel odometryNoise);

  Key GetOrInsertKey(Key newKey, double time);

  // New factor graph to add to our smoother at the next call to Optimize()
  ExpressionFactorGraph graph{};
  // New inital guesses to add to our smoother at the next call to Optimize()
  Values currentEstimate{};
  // New state timestamps to add to our smoother at the next call to Optimize()
  FixedLagSmoother::KeyTimestampMap newTimestamps{};
  // Factors to delete
  FactorIndices factorsToRemove{};
  // Log of old twists
  typedef std::map<Key, Pose3> KeyPoseDeltaMap;
  KeyPoseDeltaMap twistsFromPreviousKey{};

  // ISAM-backed fixed-lag smoother. Will marginalize out states older then a
  // given lag.
  IncrementalFixedLagSmoother smootherISAM2;

  // Current "tip" world->body estimate
  Pose3 wTb_latest;

  // keep track of our current state. State is encoded as X(uS since epoch).
  // the Key class uses the lower 56 bits for the index, and top 8 for symbol
  // 2^(64−8)÷10^6÷60÷60÷24÷365 = 2284 years, so as long as we use a sane epoch
  // we're good This will only work on 64-bit machines, but oh well. big shame.
  Key currStateIdx = X(0);
};
