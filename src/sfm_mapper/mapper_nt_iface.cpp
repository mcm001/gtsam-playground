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

#include "mapper_nt_iface.h"

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

MapperNtIface::MapperNtIface()
    : keyframeListener(nt::NetworkTableInstance::GetDefault()
                           .GetStructArrayTopic<TagDetection>("/cam/tags")
                           .Subscribe({}, nt::PubSubOptions{
                                              .pollStorage = 100,
                                              .sendAll = true,
                                              .keepDuplicates = true,
                                          })) {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

  inst.StopServer();
  inst.SetServer("10.0.0.183");
  inst.StartClient4("gtsam-mapper-meme");

  frc::SmartDashboard::PutData("GtsamMeme", &field);
}

std::map<gtsam::Key, std::vector<TagDetection>> MapperNtIface::NewKeyframes() {
  std::map<gtsam::Key, std::vector<TagDetection>> ret;

  for (const auto snapshot : keyframeListener.ReadQueue()) {
    ret[keyframe] = snapshot.value;
    keyframe++;

    // HACK - only add one snapshot per loop. need to rate limit this robot code
    // side
    break;
  }

  return ret;
}

void MapperNtIface::PublishLayout(AprilTagFieldLayout layout) {
  // todo - this destroys the relationship between ID and pose
  std::vector<Pose2d> out;

  for (const auto tag : layout.GetTags()) {
    out.push_back(tag.pose.ToPose2d());
  }

  field.GetObject("EstimatedField")->SetPoses(out);
}
