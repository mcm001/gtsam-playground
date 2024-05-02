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

#include "camera_listener.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "gtsam_utils.h"
#include "localizer.h"

using std::vector;
using namespace gtsam;

CameraListener::CameraListener(std::string rootTable, CameraConfig config,
                               std::shared_ptr<Localizer> localizer_)
    : config(config), localizer(localizer_),
      tagSub(nt::NetworkTableInstance::GetDefault()
                 .GetStructArrayTopic<TagDetection>(
                     rootTable + nt::NetworkTable::PATH_SEPARATOR_CHAR +
                     config.m_subtableName + "/input/tags")
                 .Subscribe({},
                            {
                                .pollStorage = 100,
                                .sendAll = true,
                                .keepDuplicates = true,
                            })),
      robotTcamSub(nt::NetworkTableInstance::GetDefault()
                       .GetStructTopic<frc::Transform3d>(
                           rootTable + nt::NetworkTable::PATH_SEPARATOR_CHAR +
                           config.m_subtableName + "/input/robotTcam")
                       .Subscribe({},
                                  {
                                      .pollStorage = 1,
                                      .sendAll = false,
                                      .keepDuplicates = false,
                                  })),
      pinholeIntrinsicsSub(
          nt::NetworkTableInstance::GetDefault()
              .GetDoubleArrayTopic(
                  rootTable + nt::NetworkTable::PATH_SEPARATOR_CHAR +
                  config.m_subtableName + "/input/cam_intrinsics")
              .Subscribe({},
                         {
                             .pollStorage = 1,
                             .sendAll = false,
                             .keepDuplicates = false,
                         })),
      measurementNoise(noiseModel::Isotropic::Sigma(2, config.m_pixelNoise)) {}

bool CameraListener::Update() {
  if (!localizer) {
    throw std::runtime_error("Localizer was null");
  }

  // grab the latest camera cal
  const auto last_K = pinholeIntrinsicsSub.GetAtomic();
  // if not published, time will be zero
  if (last_K.time > 0) {
    // Update calibration!
    std::vector<double> K_ = last_K.value;
    if (K_.size() != 4) {
      fmt::println("Camera {}: K of odd size {}?", config.m_subtableName,
                   K_.size());
      return false;
    }
    // assume order is [fx fy cx cy] from NT
    auto newK = Cal3_S2{K_[0], K_[1],
                        0, // no skew
                        K_[2], K_[3]};
    if (!cameraK || !cameraK->equals(newK, 1e-6)) {
      cameraK = newK;
      cameraK->print("New camera calibration");
    }
  }
  if (!cameraK) {
    fmt::println("Camera {}: no intrinsics set?", config.m_subtableName);
    return false;
  }

  // grab the latest robot-cam transform
  const auto last_rTc = robotTcamSub.GetAtomic();
  // if not published, time will be zero
  if (last_rTc.time == 0) {
    fmt::println("Camera {}: no robot-cam set?", config.m_subtableName);
    return false;
  }

  Pose3 robotTcam = Transform3dToGtsamPose3(last_rTc.value);
  // add transform to change from the wpilib/photon default (camera optical axis
  // along +x) to the standard from opencv (z along optical axis)
  robotTcam =
      robotTcam * Pose3{/*
                                     We want:
                                     x in [0,-1,0]
                                     y in [0,0,-1]
                                     z in [1,0,0]
                                     */
                        Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), Point3{0.0, 0, 0.0}};

  // Hard-coded for now -- TODO move to configurable
  // const Pose3 bodyPcamera_cam1{/*
  //                         We want:
  //                         x in [0,-1,0]
  //                         y in [0,0,-1]
  //                         z in [1,0,0]
  //                         */
  //                              Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0),

  //                              Point3{0.5, 0, 0.5}};
  // Cal3_S2 K_cam1(1000, 1000, 0, 960 / 2, 720 / 2);

  const auto tags = tagSub.ReadQueue();

  // For each tag-array in the queue
  for (const auto &tarr : tags) {
    // For each tag in this tag array
    for (const auto &t : tarr.value) {
      vector<Point2> cornersForGtsam;
      cornersForGtsam.reserve(4);
      for (const auto &c : t.corners) {
        cornersForGtsam.emplace_back(c.first, c.second);
      }

      try {
        localizer->AddTagObservation(t.id, *cameraK, robotTcam, cornersForGtsam,
                                     measurementNoise, tarr.time);
      } catch (const std::exception &e) {
        fmt::println("exception adding tag, {}", e.what());
      }
    }
  }

  return true;
}
