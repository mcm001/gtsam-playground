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

#include "helpers.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>

#include <fstream>
#include <sstream>
#include <string>
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

using namespace std;
using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::X;

void from_json(const wpi::json &json, TargetCorner &corner) {
  corner.x = json.at("x").get<double>();
  corner.y = json.at("y").get<double>();
}

void from_json(const wpi::json &json, TagDetection &tag) {
  tag.id = json.at("id").get<int>();
  tag.corners = json.at("corners").get<std::vector<TargetCorner>>();
}

map<Key, vector<TagDetection>> ParseFile() {
  map<Key, vector<TagDetection>> ret;

  std::ifstream infile("data/field_tags_2024_trim.jsonl");
  std::string line;
  Key observation_idx = 0;
  while (std::getline(infile, line)) {
    wpi::json line_json = wpi::json::parse(line);

    ret[X(observation_idx)] = line_json.get<vector<TagDetection>>();

    observation_idx++;
  }

  return ret;
}

/**
 * Estimate where our camera was at using the seed map
 */
std::optional<gtsam::Pose3>
estimateObservationPose(std::vector<TagDetection> tags,
                        frc::AprilTagFieldLayout layout) {
  static CameraMatrix calCore;
  calCore << cam_fx, 0, cam_cx, 0, cam_fy, cam_cy, 0, 0, 1;
  static DistortionMatrix calDist = DistortionMatrix::Zero();

  if (const auto worldTcam =
          photon::EstimateWorldTCam_SingleTag(tags, layout, calCore, calDist)) {
    // worldTcam->print(" >>> World2cam_singletag:\n");
    return worldTcam;
  } else {
    return std::nullopt;
  }
}

bool tagWasUsed(std::map<gtsam::Key, std::vector<TagDetection>> tags, int id) {
  for (const auto &[key, tags] : tags) {
    for (const TagDetection &tag : tags) {
      if (tag.id == id) {
        return true;
      }
    }
  }

  return false;
}
