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
using gtsam::Pose3;
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

inline const frc::AprilTagFieldLayout kDefaultLayout{
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape)};

float width = 6.5 * 25.4 / 1000.0; // 6.5in wide tag
vector<Point3> tagToCorners{
    {0, -width / 2.0, -width / 2.0},
    {0, width / 2.0, -width / 2.0},
    {0, width / 2.0, width / 2.0},
    {0, -width / 2.0, width / 2.0},
};

map<int, Pose3> worldTtags; // = TagLayoutToMap(kDefaultLayout);
} // namespace

namespace TagModel {

void SetLayout(const frc::AprilTagFieldLayout &layout) {
  worldTtags = TagLayoutToMap(layout);
}

std::optional<vector<Point3>> WorldToCorners(int id) {
  auto maybePose = worldTtags.find(id);
  if (maybePose == worldTtags.end()) {
    return std::nullopt;
  }
  Pose3 worldTtag = maybePose->second;

  vector<Point3> out(4);
  std::transform(
      tagToCorners.begin(), tagToCorners.end(), out.begin(),
      [&worldTtag](const auto &p) { return worldTtag.transformFrom(p); });

  return out;
}
} // namespace TagModel
