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

#include <memory>
#include <string>

#include <frc/geometry/Pose3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "TagDetectionStruct.h"
#include "config.h"

class Localizer;

/**
 * Set of topics to publish things to NT
 */
class DataPublisher {
public:
  DataPublisher(std::string rootTable, std::shared_ptr<Localizer> localizer);

  /**
   * Publish new data to NT
   */
  void Update();

private:
  std::shared_ptr<Localizer> localizer;

  // field-robot optimized pose
  nt::StructPublisher<frc::Pose3d> optimizedPosePub;
  // Trajectory over an arbitrary past time
  nt::StructArrayPublisher<frc::Pose3d> trajectoryHistoryPub;
  // standard deviations on rx ry rz tx ty tz
  nt::DoubleArrayPublisher stdDevPub;
};
