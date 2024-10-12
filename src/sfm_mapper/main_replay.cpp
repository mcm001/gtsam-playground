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

#include <fmt/core.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <ntcore_cpp_types.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
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
#include "helpers.h"
#include "mapper_nt_iface.h"

using std::cout;
using std::endl;
using std::map;
using std::vector;
using namespace gtsam;
using namespace noiseModel;
using symbol_shorthand::C;
using symbol_shorthand::L;
using symbol_shorthand::X;
using namespace std::chrono_literals;

static frc::AprilTagFieldLayout tagLayoutGuess =
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

// ==== Constants ====

// Global camera calibration for both estimateWorldTcam and the actual
// sfm_mapper code
const double cam_fx = 5.9938E+02;
const double cam_fy = 5.9917E+02;
const double cam_cx = 4.7950E+02;
const double cam_cy = 3.5950E+02;

Cal3_S2 camera1Cal{cam_fx, cam_fy, 0.0, cam_cx, cam_cy};

std::map<gtsam::Key, gtsam::Cal3_S2> cameraCal{
    {helpers::CameraIdxToKey(1), camera1Cal}};

// HACK: assume tag 7 is "fixed"
const std::vector<int> FIXED_TAGS{7};

// HACK: assume one camera
Pose3 worldTbody_nominal{};
gtsam::Key cameraKey{C(1)};

// Odoometry factor stdev: rad,rad,rad,m, m, m
::gtsam::SharedNoiseModel odomNoise = noiseModel::Diagonal::Sigmas(
    (Vector(6, 1) << 0.008, 0.008, 0.008, 0.004, 0.004, 0.004).finished());

// how sure we are about camera observations
Isotropic::shared_ptr cameraNoise{
    Isotropic::Sigma(2, 1.0)}; // one pixel in u and v

int main() {
  using sfm_mapper::KeyframeList;
  using sfm_mapper::OdometryList;

  auto ret{wpilog_reader::LoadDataFile("../logs/FRC_20241012_053443.wpilog",
                                       "NT:/gtsam_meme/robot_odom",
                                       "NT:/gtsam_meme/cam1/tags")};

  sfm_mapper::SfmMapper mapper{tagLayoutGuess, odomNoise, cameraNoise,
                               cameraCal, FIXED_TAGS};

  MapperNtIface ntIface;


  try {
    mapper.Optimize(sfm_mapper::OptimizerState{ret.odom, newObservations});
  } catch (std::exception *e) {
    std::cerr << e->what() << std::endl;
  }

  return 0;
}
