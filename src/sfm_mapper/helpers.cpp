#include <variant>
#include <vector>

#include "helpers.h"


#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <wpi/DataLogReader.h>
#include <wpi/DenseMap.h>

#include "TagModel.h"
#include "gtsam_utils.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>

#include "TagDetection.h"
#include <vector>

#include <frc/geometry/Twist3d.h>
#include <gtsam/inference/Symbol.h>
#include "PhotonPoseEstimator.h"

#include <fstream>
#include <sstream>
#include <string>
#include <wpi/json.h>

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
  tag.corners =
      json.at("corners").get<std::vector<TargetCorner>>();
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
std::optional<gtsam::Pose3> estimateObservationPose(std::vector<TagDetection> tags,
                                     frc::AprilTagFieldLayout layout) {
  static CameraMatrix calCore;
  calCore << 600, 0, 960/2, 0, 600, 720/2, 0, 0, 1;
  static DistortionMatrix calDist = DistortionMatrix::Zero();

  if (const auto worldTcam = photon::MultiTagOnRioStrategy(tags, layout, calCore, calDist)) {
    gtsam::Pose3 ret = Pose3dToGtsamPose3(*worldTcam);
    std::cout << "Guess for tags ";
    for (const auto& t : tags) std::cout << t.id << " ";
    std::cout << std::endl;
    ret.print("");
    return ret;
  } else {
    return std::nullopt;
  }
}