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

#include <frc/geometry/Twist3d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "helpers.h"

using namespace frc;
using namespace gtsam;
using namespace sfm_mapper;

MapperNtIface::MapperNtIface()
    : keyframeListener(
          nt::NetworkTableInstance::GetDefault()
              .GetStructArrayTopic<TagDetection>("/gtsam_meme/cam1/tags")
              .Subscribe({},
                         nt::PubSubOptions{
                             .pollStorage = 100,
                             .sendAll = true,
                             .keepDuplicates = true,
                         })),

      odomSub(nt::NetworkTableInstance::GetDefault()
                  .GetStructTopic<frc::Twist3d>("/gtsam_meme/robot_odom")
                  .Subscribe({}, {
                                     .pollStorage = 1000,
                                     .sendAll = true,
                                     .keepDuplicates = true,
                                 })) {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

  inst.StopServer();
  inst.SetServer("10.0.0.118");
  inst.StartClient4("gtsam-mapper-meme");

  frc::SmartDashboard::PutData("GtsamMeme", &field);
}

sfm_mapper::KeyframeList MapperNtIface::NewKeyframes() {
  sfm_mapper::KeyframeList ret;

  for (const auto snapshot : keyframeListener.ReadQueue()) {
    ret.push_back(sfm_mapper::KeyframeData{snapshot.time,
                                           // TODO hard-coded camera idx
                                           helpers::CameraIdxToKey(1),
                                           snapshot.value});
  }

  return ret;
}

sfm_mapper::OdometryList MapperNtIface::NewOdometryFactors() {
  const auto odom = odomSub.ReadQueue();

  sfm_mapper::OdometryList ret{};

  for (const auto &o : odom) {
    auto &twist = o.value;

    Vector6 eigenTwist;
    eigenTwist << twist.rx.to<double>(), twist.ry.to<double>(),
        twist.rz.to<double>(), twist.dx.to<double>(), twist.dy.to<double>(),
        twist.dz.to<double>();
    const Pose3 odomPoseDelta = Pose3::Expmap(eigenTwist);

    ret.push_back(sfm_mapper::OdomPoseDelta{o.time, odomPoseDelta});
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
