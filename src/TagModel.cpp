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

#include "TagModel.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "gtsam_utils.h"

using gtsam::Cal3_S2;
using gtsam::Point3;
using gtsam::Point3_;
using gtsam::Pose3;
using gtsam::Pose3_;
using gtsam::Rot3;
using std::map;
using std::vector;

namespace {

map<int, Pose3> TagLayoutToMap(const frc::AprilTagFieldLayout &layout) {
  map<int, Pose3> worldTtags;

  for (const frc::AprilTag &tag : layout.GetTags()) {
    worldTtags[tag.ID] = Pose3dToGtsamPose3(tag.pose);
  }

  return worldTtags;
}

inline const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};

float width = 6.5 * 25.4 / 1000.0; // 6.5in wide tag
const vector<Point3> tagToCorners{
    {0, -width / 2.0, -width / 2.0},
    {0, width / 2.0, -width / 2.0},
    {0, width / 2.0, width / 2.0},
    {0, -width / 2.0, width / 2.0},
};
const vector<Point3_> tagToCornersFac{
    Point3{0, -width / 2.0, -width / 2.0},
    Point3{0, width / 2.0, -width / 2.0},
    Point3{0, width / 2.0, width / 2.0},
    Point3{0, -width / 2.0, width / 2.0},
};

map<int, Pose3> worldTtags = TagLayoutToMap(kTagLayout);
} // namespace

namespace TagModel {

std::optional<gtsam::Pose3> InitialWorldToTag(int id) {
  auto maybePose = worldTtags.find(id);
  if (maybePose == worldTtags.end()) {
    return std::nullopt;
  }
  Pose3 worldTtag = maybePose->second;
  return worldTtag;
}

vector<Point3_> WorldToCornersFactor(const gtsam::Pose3_ &worldTtag) {
  vector<Point3_> out;
  out.reserve(4);

  for (int i = 0; i < 4; i++) {
    out[i] = ::gtsam::transformFrom(worldTtag, tagToCornersFac[i]);
  }

  // std::transform(tagToCornersFac.begin(), tagToCornersFac.end(), out.begin(),
  //                [&worldTtag](const auto &p) {
  //                  return ::gtsam::transformFrom(worldTtag, p);
  //                });

  return out;
}

std::optional<vector<Point3>> WorldToCorners(int id) {
  auto maybePose = worldTtags.find(id);
  if (maybePose == worldTtags.end()) {
    return std::nullopt;
  }
  Pose3 worldTtag = maybePose->second;

  vector<Point3> out;
  out.reserve(4);
  std::transform(
      tagToCorners.begin(), tagToCorners.end(), out.begin(),
      [&worldTtag](const auto &p) { return worldTtag.transformFrom(p); });

  return out;
}
} // namespace TagModel
