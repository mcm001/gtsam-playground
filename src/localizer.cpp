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

#include "localizer.h"

#include <algorithm>

#include "TagModel.h"
#include "gtsam/nonlinear/Expression.h"
#include "gtsam_utils.h"

using namespace gtsam;
using symbol_shorthand::X;

constexpr int NUM_CORNERS = 4;

Localizer::Localizer() {
  ISAM2Params parameters;
  // parameters.relinearizeThreshold = 0.01;
  // parameters.relinearizeSkip = 1;
  // parameters.cacheLinearizedFactors = false;
  // parameters.enableDetailedResults = true;
  parameters.findUnusedFactorSlots = true;
  parameters.print();

  // TODO: make sure that timestamps in units of uS doesn't cause numerical
  // precision issues
  double lag = 5 * 1e6;
  smootherISAM2 = IncrementalFixedLagSmoother(lag, parameters);

  // // And make sure to call optimize first to get values
  // TODO i killed maybe needed, idk
  // Optimize();
}

void Reset(Pose3 wTr, SharedNoiseModal noise, uint64_t timeUs) {
  currStateIdx = X(timeUs);

  smootherISAM2 = IncrementalFixedLagSmoother(smootherISAM2.smootherLag(),
                                              smootherISAM2.params());

  graph.resize(0);
  currentEstimate.clear();
  newTimestamps.clear();
  factorsToRemove.clear();

  // Anchor graph using initial pose
  graph.addPrior(X(0), wTr, posePriorNoise);
  currentEstimate.insert(X(timeUs), wTr);
  newTimestamps[X(timeUs)] = 0.0;

  wTb_latest = wTr;
}

void Localizer::AddOdometry(Pose3 poseDelta, SharedNoiseModal odometryNoise,
                            uint64_t timeUs) {
  Key newStateIdx = X(timeUs);

  // Add an odometry pose delta from our last state to our new one
  graph.emplace_shared<BetweenFactor<Pose3>>(currStateIdx, newStateIdx,
                                             poseDelta, odometryNoise);

  // And get initial guess just by composing previous pose
  wTb_latest = wTb_latest.transformPoseFrom(poseDelta);
  currentEstimate.insert(newStateIdx, wTb_latest);

  newTimestamps[newStateIdx] = timeUs;
  twistsFromPreviousKey[newStateIdx] = poseDelta;

  currStateIdx = newStateIdx;
}

Key Localizer::InsertIntoSmoother(Key lower, Key upper, Key newKey,
                                  double newTime) {
  /**
   * Goal: find the FactorIndex that connects our lower/upper key, and replace
   * it with 2 new factors and an intermediatestate
   */

  const VariableIndex &variableIndex =
      smootherISAM2.getISAM2().getVariableIndex();
  const NonlinearFactorGraph &currentFactors =
      smootherISAM2.getISAM2().getFactorsUnsafe();

  // FastMap<Key, FactorIndices>::const_iterator
  const auto &factorsConnectedToUpper = variableIndex.find(upper);
  const auto &factorsConnectedToLower = variableIndex.find(lower);

  if (factorsConnectedToUpper == variableIndex.end() ||
      factorsConnectedToUpper == variableIndex.end()) {
    // unclear what to do lol
    return 0;
  }

  // we know there is exactly one factor connecting, so do sorting-at-home
  for (const FactorIndex &idxLower : factorsConnectedToLower->second) {
    for (const FactorIndex &idxUpper : factorsConnectedToUpper->second) {
      if (idxLower == idxUpper) {
        // found our connecting factor
        FactorIndex foundIndex = idxLower;

        if (foundIndex > currentFactors.size()) {
          // TODO bail somehow
          return 0;
        }

        // Find the robot motion from lower to upper
        const auto &poseDeltaLowerToUpper = twistsFromPreviousKey.find(upper);
        if (poseDeltaLowerToUpper == twistsFromPreviousKey.end()) {
          // todo bail
          return 0;
        }

        // mark this factor for removal
        factorsToRemove.push_back(foundIndex);

        const auto totalTwist = Pose3::Logmap(poseDeltaLowerToUpper->second);
        const double t = ((double)(newKey - lower)) / ((double)(upper - lower));
        const auto twistLowerToMid = totalTwist * t;
        const auto twistMidToHigh = totalTwist - twistLowerToMid;

        // And add odometry pose deltas
        Pose3 deltaLowerToMid = Pose3::Expmap(twistLowerToMid);
        Pose3 deltaMidToHigh = Pose3::Expmap(twistLowerToMid);
        graph.emplace_shared<BetweenFactor<Pose3>>(
            lower, newKey, deltaLowerToMid, odometryNoise);
        graph.emplace_shared<BetweenFactor<Pose3>>(
            newKey, upper, deltaMidToHigh, odometryNoise);

        // and add estimates
        Pose3 currentWorldToLower =
            smootherISAM2.calculateEstimate<Pose3>(lower);
        currentEstimate.insert(
            newKey, currentWorldToLower.transformPoseFrom(deltaLowerToMid));
        newTimestamps[newKey] = newTime;
        twistsFromPreviousKey[newKey] = deltaLowerToMid;
        twistsFromPreviousKey[upper] = deltaMidToHigh;

        return newKey;
      }
    }
  }

  // TODO: bail somehow
  return 0;
}

using KeyTimeConstIt = FixedLagSmoother::KeyTimestampMap::const_iterator;
static KeyTimeConstIt FindCloser(KeyTimeConstIt left, KeyTimeConstIt right,
                                 double time) {
  double deltaLeft = time - left->second;
  double deltaRight = right->second - time;
  if (deltaLeft < deltaRight) {
    return left;
  } else {
    return right;
  }
}

Key Localizer::GetOrInsertKey(Key newKey, double time) {
  using KeyTimeMap = FixedLagSmoother::KeyTimestampMap;

  const KeyTimeMap &isamTimestamps = smootherISAM2.timestamps();
  const auto &isamEntryAfter = isamTimestamps.upper_bound(newKey);
  if (isamEntryAfter == isamTimestamps.begin()) {
    throw std::runtime_error("Timestamp is before even isam history");
  }

  // safe to do this, we checked we aren't at the start
  const auto &isamEntryBefore = std::prev(isamEntryAfter);

  if (isamEntryAfter != isamTimestamps.end() &&
      isamEntryBefore->second < time) {
    // must be fully within isam
    return FindCloser(isamEntryBefore, isamEntryAfter, time)->first;
  }

  KeyTimeMap::iterator notAddedAfter = newTimestamps.upper_bound(newKey);

  if (notAddedAfter == newTimestamps.end()) {
    throw std::runtime_error(
        "Timestamp past ISAM history, but not in yet-to-be-added");
  }

  if (notAddedAfter == newTimestamps.begin() &&
      notAddedAfter != newTimestamps.end()) {
    // check in between maybe?
    if (isamEntryBefore->second < time && time < notAddedAfter->second) {
      return FindCloser(isamEntryBefore, notAddedAfter, time)->first;
    }
    throw std::runtime_error(
        "Timestamp is before not-added but not after isam history?");
  }

  KeyTimeMap::iterator notAddedBefore = std::prev(notAddedAfter);

  if (isamEntryAfter != isamTimestamps.end() &&
      isamEntryBefore->second < time) {
    // must be fully within isam
    return FindCloser(isamEntryBefore, isamEntryAfter, time)->first;
  } else if (notAddedAfter != newTimestamps.end() &&
             notAddedBefore->second < time) {
    // must be fully within not added
    return FindCloser(notAddedBefore, notAddedAfter, time)->first;
  } else {
    // already checked in between
    throw std::runtime_error("wtf");
  }

  // // Step 1: Check if the exact time is already in our graph
  // {
  //   // Check trivial case, about to be added to the smoother
  //   KeyTimeMap::iterator keyIter = newTimestamps.find(newKey);

  //   if(keyIter != newTimestamps.end()) {
  //     return keyIter->first;
  //   }
  // }
  // {
  //   const KeyTimeMap& timestamps = smootherISAM2.timestamps();

  //   // Check trivial case, time already in the smoother
  //   KeyTimeMap::const_iterator keyIter = timestamps.find(newKey);

  //   if(keyIter != timestamps.end()) {
  //     return keyIter->first;
  //   }
  // }

  // // Step 2: check if the time is fully contained within our graph
  // {
  //   const KeyTimeMap& timestamps = smootherISAM2.timestamps();
  //   // Iterator pointing to the first element greater than key, or end().
  //   // OR: first entry greater than time
  //   const auto& keyJustAfter = timestamps.upper_bound(newKey);

  //   // Make sure new time would be before history ends
  //   if (keyJustAfter != timestamps.end()) {
  //     // Case 1 -- we are inserting a factor before the smoother's history.
  //     Best we can do is bail if (keyJustAfter == timestamps.begin()) {
  //       throw std::runtime_error("Timestamp has been marginalized out!");
  //     }

  //     // Case 2 -- fully within smoother history, So we have (in increasing
  //     time order)
  //     // keyJustBefore < now < keyJustAfter

  //     // safe to do this now
  //     const auto& keyJustBefore = std::prev(keyJustAfter);

  //     return InsertIntoSmoother(keyJustBefore->first, keyJustAfter->first,
  //     newKey, time);

  //     // // Delete the odometry factor between these two kys
  //     // // HACK: is there a less bad nested iteration way to do this?
  //     // const NonlinearFactorGraph& factors = smootherISAM2.getFactors();
  //     // for (const auto& factor : factors) {
  //     //   // Find a factor that has the before/after keys as its keys
  //     //   if (factor.find(keyJustBefore.first) != factor.end() &&
  //     factor.find(keyJustAfter.first) != factor.end() && factor.size() == 2)
  //     {
  //     //     // This is probably the odometry factor???

  //     //   }
  //     // }
  //   }

  //   // else: timestmap is after smoother history ends, hopefully the next
  //   step can deal with this
  // }

  // // Step 3: check if the timestamp is in between end of smoother and start
  // of queued factors we haven't added yet
  // {
  //   // We know time is past the end of our smoother buffer, but not sure
  //   where in our timestamp list it will fall

  //   const KeyTimeMap& timestamps = smootherISAM2.timestamps();
  //   // Iterator pointing to the first element greater than key, or end().
  //   // OR: first entry greater than time
  //   const auto& keyJustBefore = std::prev(timestamps.end());

  //   // Since newTimestamps is ordered by key, and keys increase with
  //   timestamp FixedLagSmoother::KeyTimestampMap::const_iterator keyJustAfter
  //   = newTimestamps.end();

  //   if (keyJustAfter->second > time) {
  //     // yay! Same hack wrt snap to closest factor still applies

  //     double dtToBefore = time - keyJustBefore->first;
  //     double dtToAfter = keyJustAfter->second - time;
  //     if (dtToBefore < dtToAfter) {
  //       return keyJustBefore->second;
  //     } else {
  //       return keyJustAfter->first;
  //     }
  //   } else {
  //     // else: time must be fully in our keys we're about to add
  //   }

  // }

  // // Step 4: timestamp either fully within factors we haven't added yet, or
  // is past the tip/too far in the future
  // {
  //   // TODO: somehow reverse this map since I need to look-up by timestmap,
  //   not by key
  //   // TODO HACK UGH
  //   FixedLagSmoother::KeyTimestampMap::iterator keyJustAfter =
  //   newTimestamps.upper_bound(newKey);

  //   // Make sure new time would be before history ends
  //   if (keyJustAfter != newTimestamps.end()) {
  //     // Case 1 -- we are inserting a factor between smoother and new factors
  //     (which should be covered by the above). Ignore?

  //     // Case 2 -- fully within new factor history

  //     // safe to do this now
  //     const auto& keyJustBefore = std::prev(keyJustAfter);

  //     // same justification as above
  //     double dtToBefore = time - keyJustBefore->second;
  //     double dtToAfter = keyJustAfter->second - time;
  //     if (dtToBefore < dtToAfter) {
  //       return keyJustBefore->first;
  //     } else {
  //       return keyJustAfter->first;
  //     }

  //     // TODO: implement this instead
  //     // Since we haven't added to the optimizer yet, we are good to be smart
  //     and reorder factors
  //     /*
  //     ALGORITHM:
  //     - Find the twist between a and b
  //     - delete the factor between a and b
  //     - insert a new state, c, between a and b temporally
  //     - connect a -- c -- b using twists
  //       - convert betweenfactor pose delta to a twist
  //       - lerp twist as factor of the dt between a/c/b
  //       - reconnect a-c and c-b using between factors
  //       - return c
  //     */
  //   }

  //   // else: timestmap is after smoother history AND factor to add history
  //   // drop it on the floor or smth
  //   throw std::runtime_error("Sample is too new to add??");
  // }
}

void Localizer::AddTagObservation(int tagID, Cal3_S2_ cameraCal,
                                  Pose3 robotTcamera, vector<Point2> corners,
                                  SharedNoiseModal cameraNoise,
                                  uint64_t timeUs) {
  auto worldPcorners_opt = TagModel::WorldToCorners(tagID);
  if (!worldPcorners_opt) {
    // todo return bad thing
    fmt::println("Could not find tag {} in our map!", tagID);
    return;
  }
  auto worldPcorners = worldPcorners_opt.value();

  Key newKey = X(timeUs);

  // Find where we should attach our new factors to
  Key stateAtTime = GetOrInsertKey(newKey, timeUs);

  for (size_t i = 0; i < NUM_CORNERS; i++) {
    // corner in image space
    Point2 measurement = corners[i];

    // current world->body pose
    const Pose3_ worldTbody_fac(stateAtTime);
    const auto prediction = PredictLandmarkImageLocation(
        worldTbody_fac, robotTcamera, cameraCal, worldPcorners[i]);

    graph.addExpressionFactor(prediction, measurement, cameraNoise);
  }
}

void Localizer::Optimize() {
  // fmt::println("Adding {} factors!", graph.size());

  smootherISAM2.update(graph, currentEstimate, newTimestamps, factorsToRemove);

  // reset the graph; isam wants to be fed factors to be -added-
  graph.resize(0);
  currentEstimate.clear();
  newTimestamps.clear();
  factorsToRemove.clear();

  // And grab the estimate of only the latest pose (maximize laziness)
  // Cache for use with FK prediction when adding odom factors
  wTb_latest = smootherISAM2.calculateEstimate<Pose3>(currStateIdx);
}

Matrix Localizer::GetLatestMarginals() const {
  return smootherISAM2.marginalCovariance(GetCurrStateIdx());
}

Vector6 Localizer::GetPoseComponentStdDevs() const {
  Matrix marginals = GetLatestMarginals();
  return marginals.diagonal().cwiseSqrt();
}

const vector<frc::Pose3d> Localizer::GetPoseHistory() const {
  // Plot all history, so grab the whole estimate
  Values result = smootherISAM2.calculateEstimate();

  // 5 seconds of history
  auto start = currStateIdx - (5 * 1e6);

  vector<frc::Pose3d> ret;
  ret.reserve(1000);

  for (const Values::ConstKeyValuePair &estPair : result) {
    if (estPair.key < start)
      continue;

    Pose3 est = estPair.value.cast<Pose3>();

    // auto rot = est.rotation().toQuaternion();
    // vector<double> poseEst{est.x(), est.y(), est.z(), rot.w(),
    //                             rot.x(), rot.y(), rot.z()};

    ret.emplace_back(frc::Translation3d{units::meter_t{est.x()},
                                        units::meter_t{est.y()},
                                        units::meter_t{est.z()}},
                     frc::Rotation3d{est.rotation().matrix()});
  }

  return ret;
}
