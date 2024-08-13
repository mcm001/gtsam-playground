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

#include <gtsam/base/types.h> // Basic types, constants, and compatibility functions
#include <gtsam/geometry/Pose3.h>

#include <optional>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <wpi/json.h>

#include "TagDetection.h"

const double cam_fx = 5.9938E+02;
const double cam_fy = 5.9917E+02;
const double cam_cx = 4.7950E+02;
const double cam_cy = 3.5950E+02;

void from_json(const wpi::json &json, TargetCorner &corner);
void from_json(const wpi::json &json, TagDetection &tag);
std::map<gtsam::Key, std::vector<TagDetection>> ParseFile();
std::optional<gtsam::Pose3> estimateWorldTcam(std::vector<TagDetection> tags,
                                              frc::AprilTagFieldLayout layout);
bool tagWasUsed(std::map<gtsam::Key, std::vector<TagDetection>> tags, int id);
