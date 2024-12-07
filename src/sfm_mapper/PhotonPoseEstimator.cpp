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

#include "PhotonPoseEstimator.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <units/math.h>
#include <units/time.h>

#include "TagModel.h"
#include "gtsam_utils.h"

#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include <opencv2/core/eigen.hpp>

namespace photon {

namespace detail {
cv::Point3d ToPoint3d(const frc::Translation3d &translation);
std::optional<std::array<cv::Point3d, 4>>
CalcTagCorners(int tagID, const frc::AprilTagFieldLayout &aprilTags);
frc::Pose3d ToPose3d(const cv::Mat &tvec, const cv::Mat &rvec);
cv::Point3d TagCornerToObjectPoint(units::meter_t cornerX,
                                   units::meter_t cornerY, frc::Pose3d tagPose);
} // namespace detail

std::optional<std::array<cv::Point3d, 4>>
detail::CalcTagCorners(int tagID, const frc::AprilTagFieldLayout &aprilTags) {
  if (auto tagPose = aprilTags.GetTagPose(tagID); tagPose.has_value()) {
    return std::array{TagCornerToObjectPoint(-3_in, -3_in, *tagPose),
                      TagCornerToObjectPoint(+3_in, -3_in, *tagPose),
                      TagCornerToObjectPoint(+3_in, +3_in, *tagPose),
                      TagCornerToObjectPoint(-3_in, +3_in, *tagPose)};
  } else {
    return std::nullopt;
  }
}

cv::Point3d detail::ToPoint3d(const frc::Translation3d &translation) {
  return cv::Point3d(-translation.Y().value(), -translation.Z().value(),
                     +translation.X().value());
}

cv::Point3d detail::TagCornerToObjectPoint(units::meter_t cornerX,
                                           units::meter_t cornerY,
                                           frc::Pose3d tagPose) {
  frc::Translation3d cornerTrans =
      tagPose.Translation() +
      frc::Translation3d(0.0_m, cornerX, cornerY).RotateBy(tagPose.Rotation());
  return ToPoint3d(cornerTrans);
}

frc::Pose3d detail::ToPose3d(const cv::Mat &tvec, const cv::Mat &rvec) {
  using namespace frc;
  using namespace units;

  // cv::Mat R;
  // cv::Rodrigues(rvec, R); // R is 3x3
  // R = R.t();                 // rotation of inverse
  // cv::Mat tvecI = -R * tvec; // translation of inverse

  Eigen::Matrix<double, 3, 1> tv;
  tv[0] = tvec.at<double>(0, 0);
  tv[1] = tvec.at<double>(1, 0);
  tv[2] = tvec.at<double>(2, 0);
  Eigen::Matrix<double, 3, 1> rv;
  rv[0] = rvec.at<double>(0, 0);
  rv[1] = rvec.at<double>(1, 0);
  rv[2] = rvec.at<double>(2, 0);

  return Pose3d(Translation3d(meter_t{tv[0]}, meter_t{tv[1]}, meter_t{tv[2]}),
                Rotation3d(rv));
}

std::optional<gtsam::Pose3>
MultiTagOnRioStrategy(std::vector<TagDetection> targets,
                      frc::AprilTagFieldLayout aprilTags,
                      std::optional<CameraMatrix> camMat,
                      std::optional<DistortionMatrix> distCoeffs) {
  using namespace frc;

  if (!camMat || !distCoeffs) {
    return std::nullopt;
  }

  // List of corners mapped from 3d space (meters) to the 2d camera screen
  // (pixels).
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;

  // Add all target corners to main list of corners
  for (auto target : targets) {
    int id = target.GetFiducialId();
    if (auto const tagCorners = detail::CalcTagCorners(id, aprilTags);
        tagCorners.has_value()) {
      auto const targetCorners = target.corners;
      for (size_t cornerIdx = 0; cornerIdx < 4; ++cornerIdx) {
        imagePoints.emplace_back(targetCorners[cornerIdx].x,
                                 targetCorners[cornerIdx].y);
        objectPoints.emplace_back((*tagCorners)[cornerIdx]);
      }
    }
  }

  // We should only do multi-tag if at least 2 tags (* 4 corners/tag)
  if (imagePoints.size() < 4) {
    return std::nullopt;
  }

  // Output mats for results
  cv::Mat const rvec(3, 1, cv::DataType<double>::type);
  cv::Mat const tvec(3, 1, cv::DataType<double>::type);

  {
    cv::Mat cameraMatCV(camMat->rows(), camMat->cols(), CV_64F);
    cv::eigen2cv(*camMat, cameraMatCV);
    cv::Mat distCoeffsMatCV(distCoeffs->rows(), distCoeffs->cols(), CV_64F);
    cv::eigen2cv(*distCoeffs, distCoeffsMatCV);

    cv::solvePnP(objectPoints, imagePoints, cameraMatCV, distCoeffsMatCV, rvec,
                 tvec, false, cv::SOLVEPNP_SQPNP);
  }

  std::cout << "Estimate rvec " << rvec << " tvec " << tvec << std::endl;

  const Pose3d pose = detail::ToPose3d(tvec, rvec);

  return Pose3dToGtsamPose3(pose).inverse();
}

std::optional<gtsam::Pose3>
EstimateWorldTCam_SingleTag(std::vector<TagDetection> result,
                            frc::AprilTagFieldLayout aprilTags,
                            std::optional<CameraMatrix> camMat,
                            std::optional<DistortionMatrix> distCoeffs) {
  if (!camMat || !distCoeffs || !result.size()) {
    return std::nullopt;
  }

  auto tagToUse = result[0];

  // List of corners mapped from 3d space (meters) to the 2d camera screen
  // (pixels).
  /*
    ⟶ +X  3 ----- 2
    |      |       |
    V      |       |
    +Y     0 ----- 1

    In object space, we have this order, with origin at the center and +X out of
    the tag ^ +Z          3 --- 2 |             |     |
    -----> +Y     0 --- 1
  */
  auto TagCorner = [](units::meter_t x, units::meter_t y) {
    // Tag coordinate system
    return cv::Point3f{0, (float)x.to<double>(), (float)y.to<double>()};
  };
  std::vector<cv::Point3f> objectPoints{
      TagCorner(-3_in, -3_in), TagCorner(+3_in, -3_in), TagCorner(+3_in, +3_in),
      TagCorner(-3_in, +3_in)};

  std::vector<cv::Point2f> imagePoints;
  for (const auto &corner : tagToUse.corners) {
    imagePoints.emplace_back(corner.x, corner.y);
  }

  // eigen/cv marshalling
  cv::Mat cameraMatCV(camMat->rows(), camMat->cols(), CV_64F);
  cv::eigen2cv(*camMat, cameraMatCV);
  cv::Mat distCoeffsMatCV(distCoeffs->rows(), distCoeffs->cols(), CV_64F);
  cv::eigen2cv(*distCoeffs, distCoeffsMatCV);

  // actually do solvepnp
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::solvePnP(objectPoints, imagePoints, cameraMatCV, distCoeffsMatCV, rvec,
               tvec);

  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rvec, tvec, cameraMatCV, distCoeffsMatCV,
                    projectedPoints);

  // std::cout << "rvec: " << rvec << std::endl;
  // std::cout << "tvec: " << tvec << std::endl;
  // for (unsigned int i = 0; i < projectedPoints.size(); ++i) {
  //   std::cout << "Image point: " << imagePoints[i] << " Projected to "
  //             << projectedPoints[i] << std::endl;
  // }

  if (const auto w2tag = TagModel::worldToTag(tagToUse.id)) {
    auto camToTag = Pose3dToGtsamPose3(detail::ToPose3d(tvec, rvec));
    auto tag2cam = camToTag.inverse();
    // std::cout << " > world2tag\n" << *w2tag << std::endl;
    // std::cout << " > cam2tag\n" << camToTag << std::endl;
    // std::cout << " > tag2cam\n" << tag2cam << std::endl;

    return (*w2tag).transformPoseFrom(tag2cam);
  } else {
    return std::nullopt;
  }
}

} // namespace photon
