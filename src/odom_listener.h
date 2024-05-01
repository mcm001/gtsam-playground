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

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Twist3d.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "TagDetectionStruct.h"
#include "config.h"

class Localizer;

class OdomListener {
public:
  OdomListener(LocalizerConfig config, std::shared_ptr<Localizer> localizer);

  /**
   * Add all new odometry factors to the localizer
   */
  bool Update();

private:
  // Initial guess, used for the first Optimize we ever do to keep us in the
  // right ballpark
  nt::StructSubscriber<frc::Pose3d> initialGuessSub;
  bool hasInitialGuess = false;

  LocalizerConfig config;

  std::shared_ptr<Localizer> localizer;

  nt::StructSubscriber<frc::Twist3d> odomSub;

  ::gtsam::SharedNoiseModel odomNoise;
  ::gtsam::SharedNoiseModel priorNoise;
};
