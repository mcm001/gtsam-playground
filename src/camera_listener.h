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

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <optional>
#include <string>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "TagDetectionStruct.h"
#include "config.h"
#include "gtsam_utils.h"

class CameraListener {
public:
  CameraListener(std::string rootTable, CameraConfig config);

  /**
   * If all required info (eg camera calibration, robot-cam offset) has been
   * recieved and we're ready to go
   */
  bool ReadyToOptimize();

  /**
   * Add all new camera observations to the localizer
   */
  std::vector<CameraVisionObservation> Update();

private:
  // Camera (pinhole) calibration coefficients
  std::optional<gtsam::Cal3_S2> cameraK;
  // Camera offset
  std::optional<::gtsam::Pose3> robotTcamera;

  CameraConfig config;

  // Tag detection messages
  nt::StructArraySubscriber<TagDetection> tagSub;
  // Robot->this particular camera
  nt::StructSubscriber<frc::Transform3d> robotTcamSub;
  // Camera calibration; assume all pixel inputs are already undistorted
  nt::DoubleArraySubscriber pinholeIntrinsicsSub;

  ::gtsam::noiseModel::Isotropic::shared_ptr measurementNoise;
};
