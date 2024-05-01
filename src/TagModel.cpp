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

inline const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};

float width = 6.5 * 25.4 / 1000.0; // 6.5in wide tag
vector<Point3> tagToCorners{
    {0, -width / 2.0, -width / 2.0},
    {0, width / 2.0, -width / 2.0},
    {0, width / 2.0, width / 2.0},
    {0, -width / 2.0, width / 2.0},
};

map<int, Pose3> worldTtags = TagLayoutToMap(kTagLayout);
} // namespace

namespace TagModel {

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

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

namespace {
using namespace cv;
// Matx case, from eigen.hpp
template <typename _Tp, int _rows, int _cols, int _options, int _maxRows,
          int _maxCols>
static inline void eigen2cv(
    const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &src,
    Matx<_Tp, _rows, _cols> &dst) {
  if (!(src.Flags & Eigen::RowMajorBit)) {
    dst = Matx<_Tp, _cols, _rows>(static_cast<const _Tp *>(src.data())).t();
  } else {
    dst = Matx<_Tp, _rows, _cols>(static_cast<const _Tp *>(src.data()));
  }
}

// Matx case
template <typename _Tp, int _rows, int _cols, int _options, int _maxRows,
          int _maxCols>
static inline void
cv2eigen(const Matx<_Tp, _rows, _cols> &src,
         Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &dst) {
  if (!(dst.Flags & Eigen::RowMajorBit)) {
    const Mat _dst(_cols, _rows, traits::Type<_Tp>::value, dst.data(),
                   (size_t)(dst.outerStride() * sizeof(_Tp)));
    transpose(src, _dst);
  } else {
    const Mat _dst(_rows, _cols, traits::Type<_Tp>::value, dst.data(),
                   (size_t)(dst.outerStride() * sizeof(_Tp)));
    Mat(src).copyTo(_dst);
  }
}

} // namespace

Pose3 EstimateFieldToRobot(std::vector<TagDetection> tags, Cal3_S2 cal) {

  // List of corners mapped from 3d space (meters) to the 2d camera screen
  // (pixels).
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;

  // for each tagt
  for (const TagDetection &tag : tags) {
    auto corners = TagModel::WorldToCorners(tag.id);
    // only add if we found the tag in the map
    if (corners) {
      for (auto c : corners.value()) {
        objectPoints.push_back(cv::Point3f(c[0], c[1], c[2]));
      }
      for (auto i : tag.corners) {
        imagePoints.emplace_back(i.first, i.second);
      }
    }
  }

  // Output mats for results
  cv::Matx31d rvec;
  cv::Matx31d tvec;

  auto calData = cal.K();
  cv::Matx33d camMat;
  eigen2cv(calData, camMat);

  cv::solvePnP(objectPoints, imagePoints, camMat,
               cv::Mat::zeros(1, 5, CV_64FC1), rvec, tvec, false,
               cv::SOLVEPNP_SQPNP);

  Eigen::Vector3d rvecEig;
  Eigen::Vector3d tvecEig;
  cv2eigen(rvec, rvecEig);
  cv2eigen(tvec, tvecEig);

  return Pose3(Rot3::Rodrigues(rvecEig), Point3(tvecEig));
}
