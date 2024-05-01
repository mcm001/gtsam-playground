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

#include "gtsam_utils.h"

using namespace gtsam;

Pose3 FrcToGtsamPose3(frc::Pose3d pose) {
  const auto q = pose.Rotation().GetQuaternion();
  return Pose3{Rot3(q.W(), q.X(), q.Y(), q.Z()),
               Point3(pose.X().to<double>(), pose.Y().to<double>(),
                      pose.Z().to<double>())};
}

gtsam::Point2_ PredictLandmarkImageLocation(gtsam::Pose3_ worldTbody_fac,
                                            gtsam::Pose3 bodyPcamera,
                                            gtsam::Cal3_S2_ cameraCal,
                                            gtsam::Point3 worldPcorner) {
  using namespace gtsam;

  // world->camera pose as a composition of world->body factory and
  // body->camera factor
  const Pose3_ worldTcamera_fac =
      Pose3_(worldTbody_fac, &Pose3::transformPoseFrom, Pose3_(bodyPcamera));
  // Camera->tag corner vector
  const Point3_ camPcorner = transformTo(worldTcamera_fac, worldPcorner);
  // project from vector down to pinhole location, then uncalibrate to pixel
  // locations
  const Point2_ prediction =
      uncalibrate<Cal3_S2>(cameraCal, project(camPcorner));

  return prediction;
}

frc::Pose3d GtsamToFrcPose3d(gtsam::Pose3 pose) {
  return frc::Pose3d {
    frc::Translation3d{units::meter_t{pose.x()},
                       units::meter_t{pose.y()},
                       units::meter_t{pose.z()}},
    frc::Rotation3d{pose.rotation().matrix()});
  }
}
