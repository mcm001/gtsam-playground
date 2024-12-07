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

#include "wpical.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/expressions.h>

#include <optional>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include <opencv2/core/eigen.hpp>

using namespace wpical;
using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::X;

using CameraMatrix = Eigen::Matrix<double, 3, 3>;
using DistortionMatrix = Eigen::Matrix<double, 8, 1>;

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

Pose3 Pose3dToGtsamPose3(frc::Pose3d pose) {
  const auto q = pose.Rotation().GetQuaternion();
  return Pose3{Rot3(q.W(), q.X(), q.Y(), q.Z()),
               Point3(pose.X().to<double>(), pose.Y().to<double>(),
                      pose.Z().to<double>())};
}

frc::Pose3d GtsamToFrcPose3d(gtsam::Pose3 pose) {
  return frc::Pose3d{frc::Translation3d{units::meter_t{pose.x()},
                                        units::meter_t{pose.y()},
                                        units::meter_t{pose.z()}},
                     frc::Rotation3d{pose.rotation().matrix()}};
}

gtsam::Point2_
PredictLandmarkImageLocationFactor(gtsam::Pose3_ worldTcamera_fac,
                                   gtsam::Cal3_S2_ cameraCal,
                                   gtsam::Point3_ worldPcorner) {
  // Tag corner 3D position in the camera frame
  const Point3_ camPcorner = transformTo(worldTcamera_fac, worldPcorner);
  // project from vector down to pinhole location, then uncalibrate to pixel
  // locations
  const Point2_ prediction =
      uncalibrate<Cal3_S2>(cameraCal, project(camPcorner));

  return prediction;
}

// HACK - assume 6.5in wide tag
float width = 6.5 * 25.4 / 1000.0;
std::vector<Point3> tagToCorners{
    {0, -width / 2.0, -width / 2.0},
    {0, width / 2.0, -width / 2.0},
    {0, width / 2.0, width / 2.0},
    {0, -width / 2.0, width / 2.0},
};
std::vector<Point3_> WorldToCornersFactor(Pose3_ worldTtag) {
  std::vector<Point3_> out;
  for (const auto &p : tagToCorners) {
    out.push_back(transformFrom(worldTtag, p));
  }

  return out;
}

// TODO HACK fill in
std::map<int, gtsam::Pose3> worldTtags;
std::optional<gtsam::Pose3> worldToTag(int id) {
  if (auto it = worldTtags.find(id); it != worldTtags.end()) {
    return it->second;
  } else {
    return std::nullopt;
  }
}

frc::Pose3d ToPose3d(const cv::Mat &tvec, const cv::Mat &rvec) {
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

  if (const auto w2tag = worldToTag(tagToUse.id)) {
    auto camToTag = Pose3dToGtsamPose3(ToPose3d(tvec, rvec));
    auto tag2cam = camToTag.inverse();
    // std::cout << " > world2tag\n" << *w2tag << std::endl;
    // std::cout << " > cam2tag\n" << camToTag << std::endl;
    // std::cout << " > tag2cam\n" << tag2cam << std::endl;

    return (*w2tag).transformPoseFrom(tag2cam);
  } else {
    return std::nullopt;
  }
}

std::optional<gtsam::Pose3> estimateWorldTcam(std::vector<TagDetection> tags,
                                              frc::AprilTagFieldLayout layout,
                                              Cal3_S2 cal) {
  CameraMatrix calCore = cal.K();
  DistortionMatrix calDist = DistortionMatrix::Zero();

  if (const auto worldTcam =
          EstimateWorldTCam_SingleTag(tags, layout, calCore, calDist)) {
    // worldTcam->print(" >>> World2cam_singletag:\n");

    return worldTcam;
  } else {
    return std::nullopt;
  }
}

CalResult OptimizeLayout(
    const frc::AprilTagFieldLayout &tagLayoutGuess,
    const KeyframeMap &keyframes, gtsam::Cal3_S2 cal,
    const std::map<int32_t, std::pair<gtsam::Pose3, gtsam::SharedNoiseModel>>
        &fixedTags,
    const gtsam::SharedNoiseModel cameraNoise) {

  ExpressionFactorGraph graph;

  // constrain fixed(ish) tags - future work can investigate partial pose priors
  for (const auto &[tagId, info] : fixedTags) {
    graph.addPrior(L(tagId), std::get<0>(info), std::get<1>(info));
  }

  // Add all our tag observation factors
  for (const auto &[stateKey, tags] : keyframes) {
    for (const TagDetection &tag : tags) {
      auto worldPcorners = WorldToCornersFactor(L(tag.id));

      // add each tag corner
      constexpr int NUM_CORNERS = 4;
      for (size_t i = 0; i < NUM_CORNERS; i++) {
        // Decision variable - where our camera is in the world
        const Pose3_ worldTcamera_fac(stateKey);
        // Where we'd predict the i'th corner of the tag to be
        const auto prediction = PredictLandmarkImageLocationFactor(
            worldTcamera_fac, cal, worldPcorners[i]);
        // where we saw the i'th corner in the image
        Point2 measurement = {tag.corners[i].x, tag.corners[i].y};
        // Add this prediction/measurement pair to our graph
        graph.addExpressionFactor(prediction, measurement, cameraNoise);
      }
    }
  }

  // Initial guess for our optimizer. Needs to be in the right ballpark, but
  // accuracy doesn't super matter
  Values initial;

  // Guess for all camera poses based on tag layout JSON
  for (const auto &[stateKey, tags] : keyframes) {
    auto worldTcam_guess = estimateWorldTcam(tags, tagLayoutGuess, cal);
    if (!worldTcam_guess) {
      std::cerr << "Can't guess pose of camera for observation " << stateKey
                << std::endl;
    } else {
      initial.insert<Pose3>(stateKey, *worldTcam_guess);
    }
  }

  // Guess for tag locations = tag layout json
  for (const frc::AprilTag &tag : tagLayoutGuess.GetTags()) {
    if (tagWasUsed(keyframes, tag.ID)) {
      initial.insert(L(tag.ID), Pose3dToGtsamPose3(tag.pose));
    }
  }

  /* Optimize the graph and print results */
  std::cout << "==========================\ninitial error = "
            << graph.error(initial) << std::endl;
  auto start = std::chrono::steady_clock::now();

  DoglegParams params;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  // params.relativeErrorTol = 1e-3;
  // params.absoluteErrorTol = 1e-3;

  // Create initial optimizer
  DoglegOptimizer optimizer{graph, initial, params};

  // Run full optimization until convergence.
  Values result;
  try {
    result = optimizer.optimize();
  } catch (std::exception *e) {
    std::cerr << e->what();
    return {};
  }

  auto end = std::chrono::steady_clock::now();
  auto dt = end - start;
  long long microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(dt).count();

  std::cout << "\n===== Converged in " << optimizer.iterations()
            << " iterations (" << microseconds << " uS) with final error "
            << optimizer.error() << " ======" << std::endl;

  {
    std::stringstream ss;
    ss << "tag_map_" << result.size() << ".dot";
    graph.saveGraph(ss.str(), result);
  }

  CalResult ret;

  {
    gtsam::Marginals marginals(graph, result);
    std::vector<frc::AprilTag> tags;

    std::cout << "Results:" << std::endl;
    for (auto [key, value] : result) {
      std::cout << "\n========= Key " << gtsam::Symbol(key) << " ==========\n";

      // Assume all our keys are pose3 factors. lol.
      auto est = GtsamToFrcPose3d(result.at<gtsam::Pose3>(key));
      fmt::println("Estimated pose:");
      fmt::println("Translation: x={:.2f} y={:.2f} z={:.2f}", est.X(), est.Y(),
                   est.Z());
      fmt::println("Rotation: W={:.3f} X={:.3f} Y={:.3f} Z={:.3f}",
                   est.Rotation().GetQuaternion().W(),
                   est.Rotation().GetQuaternion().X(),
                   est.Rotation().GetQuaternion().Y(),
                   est.Rotation().GetQuaternion().Z());

      // Covariance is the variance of x_i with x_i - stddev is sqrt(var)
      std::cout << "Marginal covariance (r t):" << std::endl
                << marginals.marginalCovariance(key).diagonal().cwiseSqrt()
                << std::endl;

      // todo - track all tag keys instead of this hack
      if (key >= L(0) && key <= L(2000)) {
        tags.push_back(frc::AprilTag{static_cast<int>(key - L(0)), est});
      }
    }

    frc::AprilTagFieldLayout layout{tags, 16.541_m, 8.211_m};
    // ntIface.PublishLayout(layout);
  }

  return ret;
}
