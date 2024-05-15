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

#include <gtest/gtest.h>

#include "localizer.h"

using namespace gtsam;

TEST(LocalizerTest, LatencyCompensate) {
  /*
  We want:
  x in [0,-1,0]
  y in [0,0,-1]
  z in [1,0,0]
  */
  const Pose3 bodyPcamera{Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0),
                          Point3{0.5, 0, 0.5}};

  // Cal3_S2 K(90, 960, 720);
  Cal3_S2 K(1000, 1000, 0, 960 / 2, 720 / 2);

  // setup noise using fake numbers
  // Pixel noise, in u,v coordinates
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 2.0);

  // Noise on the prior factor we use to anchor the first pose.
  // TODO: If we initialize with enough measurements, we might be able to
  // delete this prior?
  Vector6 sigmas;
  sigmas << Vector3::Constant(0.1), Vector3::Constant(0.3);
  auto posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

  // odometry noise
  Vector6 odomSigma;
  odomSigma << Vector3::Constant(0.001), Vector3::Constant(0.05);
  auto odometryNoise = noiseModel::Diagonal::Sigmas(odomSigma);

  Cal3_S2_ cal(K);

  auto localizer = Localizer();

  localizer.Reset(Pose3(), posePriorNoise, 5 * 1000);
  localizer.AddOdometry(OdometryObservation{
      100 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.AddOdometry(OdometryObservation{
      200 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.AddOdometry(OdometryObservation{
      300 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.AddOdometry(OdometryObservation{
      400 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});
  localizer.Optimize();
  auto pose = localizer.GetLatestWorldToBody();
  pose.print("Pose: ");

  CameraVisionObservation obs{240000,
                              8,
                              {
                                  {414, 166},
                                  {457, 165},
                                  {457, 122},
                                  {412, 122},
                              },
                              cal,
                              Pose3(),
                              measurementNoise};

  // add vision to within isam's history
  localizer.AddTagObservation(obs);

  localizer.Optimize();
  pose = localizer.GetLatestWorldToBody();
  localizer.Print();

  // add but don't optimize
  localizer.AddOdometry(OdometryObservation{
      500 * 1000, Pose3{Rot3{}, Point3{1, 0, 0}}, odometryNoise});

  obs = {460000,
         8,
         {
             {414, 166},
             {457, 165},
             {457, 122},
             {412, 122},
         },
         cal,
         Pose3(),
         measurementNoise};

  localizer.AddTagObservation(obs);
  localizer.Optimize();
  pose = localizer.GetLatestWorldToBody();
  localizer.Print();
}
