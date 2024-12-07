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

#include <gtsam/inference/Symbol.h>

#include <map>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "TagDetection.h"
#include "TagDetectionStruct.h"
#include "wpical/Pose3WithVariance.h"

class MapperNtIface {

public:
  MapperNtIface();

  std::map<gtsam::Key, std::vector<TagDetection>> NewKeyframes();
  void PublishResult(frc::AprilTagFieldLayout layout, std::vector<Pose3WithVariance> tags, std::vector<Pose3WithVariance> camera);

private:
  nt::StructArraySubscriber<TagDetection> keyframeListener;

  nt::StructArrayPublisher<Pose3WithVariance> tagPoseCovPub;
  nt::StructArrayPublisher<Pose3WithVariance> cameraPoseCovPub;

  frc::Field2d field{};

  gtsam::Key keyframe = gtsam::symbol_shorthand::X(0);
};
