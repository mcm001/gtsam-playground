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

#include <gtsam/linear/NoiseModel.h>

#include <optional>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "TagDetectionStruct.h"
#include "config.h"

class Localizer;

class CameraListener {
public:
  CameraListener(std::string_view rootTable, CameraConfig config,
                 std::shared_ptr<Localizer> localizer);

  /**
   * Add all new camera observations to the localizer
   */
  void Update();

private:
  // Camera (pinhole) calibration coefficients
  std::optional<Cal3_S2> cameraK;

  CameraConfig config;

  std::shared_ptr<Localizer> localizer;

  // Tag detection messages
  nt::StructArraySubscriber<TagDetection> tagSub;
  // Robot->this particular camera
  nt::StructArraySubscriber<frc::Transform3d> robotTCamSub;
  // Camera calibration; assume all pixel inputs are already undistorted
  nt::DoubleArraySubscriber pinholeIntrinsicsSub;

  ::gtsam::noiseModel::Isotropic::shared_ptr measurementNoise;
  ::gtsam::Pose3 robotTcamera;
};
