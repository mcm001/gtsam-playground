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

#include <iostream>
#include <thread>

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <frc/geometry/struct/Twist3dStruct.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "TagDetectionStruct.h"
#include "TagModel.h"
#include "config.h"
#include "gtsam_utils.h"
#include "localizer.h"
#include "odom_listener.h"
#include "camera_listener.h"
#include <shared_ptr.h>
#include "data_publisher.h"

using namespace gtsam;
using std::vector;

class LocalizerRunner {
private:
  std::shared_ptr<Localizer> localizer;
  OdomListener odomListener;
  DataPublisher dataPublisher;
  std::vector<CameraListener> cameraListeners;

public:
  LocalizerRunner(LocalizerConfig config) 
    : localizer(Localizer()),
     odomListener{config.rootTableName, localizer}, dataPublisher(config.rootTableName, localizer) {

      cameraListeners.reserve(config.cameras.size());
      for (const CameraConfig& camCfg : config.cameras) {
        cameraListeners.emplace_back(config.rootTableName, camCfg, localizer);
      }
  }

  void Update() {
    odomListener.Update();

    for (const auto& cam : cameraListeners) {
      cam.Update();
    }

    localizer->Optimize();

    dataPublisher.Update();
  }
};

int main(int argc, char **argv) {
  using namespace std::chrono_literals;

  LocalizerConfig config = ParseConfig("test/resources/good_config.json");
  config.print();

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopServer();
  inst.SetServer("192.168.1.226");
  inst.StartClient4("gtsam-meme");

  LocalizerRunner runner(config);

  while (true) {
    runner.Update();

    std::this_thread::sleep_for(100ms);
  }

  // // Attach listener
  // nt::StructArrayTopic<TagDetection> tagTopic =
  //     inst.GetStructArrayTopic<TagDetection>("/cam/tags");
  // auto tagSub = tagTopic.Subscribe({}, {
  //                                          .pollStorage = 100,
  //                                          .sendAll = true,
  //                                          .keepDuplicates = true,
  //                                      });

  // nt::StructTopic<frc::Twist3d> odomTopic = inst.GetStructTopic<frc::Twist3d>(
  //     "/ReplayOutputs/Swerve/Odometry/WheelOnlyTwist");
  // auto odomSub = odomTopic.Subscribe({}, {
  //                                            .pollStorage = 100,
  //                                            .sendAll = true,
  //                                            .keepDuplicates = true,
  //                                        });

  // auto poseEstimatePub =
  //     inst.GetDoubleArrayTopic(
  //             "/SmartDashboard/VisionSystemSim-main/Sim Field/Gtsam Robot")
  //         .Publish({.sendAll = true, .keepDuplicates = true});
  // auto observedCornersPub =
  //     inst.GetDoubleArrayTopic("/cam/gtsam_seen_corners")
  //         .Publish({.sendAll = true, .keepDuplicates = true});
  // auto predictedCornersPub =
  //     inst.GetDoubleArrayTopic("/cam/gtsam_predicted_corners")
  //         .Publish({.sendAll = true, .keepDuplicates = true});
  // auto trajectoryPub = inst.GetStructArrayTopic<frc::Pose3d>("/cam/gtsam_traj")
  //                          .Publish({.sendAll = true, .keepDuplicates = true});
  // auto dtPublisher = inst.GetDoubleTopic("/cam/update_dt_ms").Publish();
  // // standard deviations on rx ry rz tx ty tz
  // auto stdevPub = inst.GetDoubleArrayTopic("/cam/std_dev").Publish();

  // // Assume robot isn't moving to get initial guess

  // Pose3 initialEstimate;
  // {
  //   auto initialGuessSub =
  //       inst.GetStructTopic<frc::Pose3d>("/robot/multi_tag_pose")
  //           .Subscribe({}, {
  //                              .pollStorage = 100,
  //                              .sendAll = true,
  //                              .keepDuplicates = true,
  //                          });

  //   while (true) {
  //     auto guessArr = initialGuessSub.ReadQueue();
  //     if (!guessArr.empty()) {
  //       initialEstimate = FrcToGtsamPose3(guessArr.back().value);
  //       initialEstimate.print("Initial guess");
  //       break;
  //     } else {
  //       fmt::println("No initial guess yet, sleeping");
  //       std::this_thread::sleep_for(200ms);
  //     }
  //   }
  // }

  // Localizer localizer = CreateLocalizer(config.cameras.at(0), bodyPcamera_cam1,
  //                                       K_cam1, initialEstimate);

  // // hacky eventloop
  // while (true) {
  //   const auto start = std::chrono::steady_clock::now();

  //   const auto tags = tagSub.ReadQueue();
  //   const auto odom = odomSub.ReadQueue();

  //   for (auto o : odom) {
  //     auto twist = o.value;

  //     Pose3 odomPoseDelta = Pose3::Expmap(
  //         (Vector6() << twist.rx.to<double>(), twist.ry.to<double>(),
  //          twist.rz.to<double>(), twist.dx.to<double>(), twist.dy.to<double>(),
  //          twist.dz.to<double>())
  //             .finished());

  //     try {
  //       localizer.AddOdometry(odomPoseDelta, o.time);
  //     } catch (std::exception e) {
  //       fmt::println("whoops, {}", e.what());
  //     }
  //   }

  //   for (const auto &tarr : tags) {
  //     std::vector<double> corners;
  //     std::vector<double> seenCorners;
  //     // fmt::println("Update from {}:", tarr.time);

  //     for (const auto &tag : tarr.value) {
  //       {
  //         vector<Point2> cornersForGtsam;
  //         for (const auto &c : tag.corners) {
  //           cornersForGtsam.emplace_back(c.first, c.second);
  //         }

  //         try {
  //           localizer.AddTagObservation(tag.id, cornersForGtsam,
  //                                       tarr.time);
  //         } catch (std::exception e) {
  //           fmt::println("whoops tag, {}", e.what());
  //         }
  //       }

  //       // Publish reprojected corners
  //       try {
  //         auto worldPtag_opt = TagModel::WorldToCorners(tag.id);
  //         if (worldPtag_opt) {
  //           auto worldPtag = worldPtag_opt.value();
  //           auto worldTrobot = localizer.GetLatestWorldToBody();
  //           PinholeCamera<Cal3_S2> cam(worldTrobot * bodyPcamera_cam1, K_cam1);
  //           for (auto worldPcorner : worldPtag) {
  //             const Point2 prediction = cam.project2(worldPcorner);
  //             corners.push_back(prediction.x());
  //             corners.push_back(prediction.y());
  //           }
  //           for (auto seenCorner : tag.corners) {
  //             seenCorners.push_back(seenCorner.first);
  //             seenCorners.push_back(seenCorner.second);
  //           }
  //         }
  //       } catch (std::exception e) {
  //         fmt::println("whoops reproj, {}", e.what());
  //       }
  //     }

  //     predictedCornersPub.Set(corners, tarr.time);
  //     observedCornersPub.Set(seenCorners, tarr.time);
  //   }

  //   // Update so we can publish a new pose estimate every tick
  //   localizer.Optimize();
  //   auto est = localizer.GetLatestWorldToBody();
  //   auto rot = est.rotation().toQuaternion();
  //   std::vector<double> poseEst{est.x(), est.y(), est.z(), rot.w(),
  //                               rot.x(), rot.y(), rot.z()};
  //   poseEstimatePub.Set(poseEst);

  //   {
  //     // std::cout << "Latest mariginal\n" << localizer.GetLatestMarginals()
  //     // <<
  //     // std::endl; std::cout << "Latest stdev\n" <<
  //     // localizer.GetPoseComponentStdDevs() << std::endl;
  //     auto mat = localizer.GetPoseComponentStdDevs();
  //     std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
  //     stdevPub.Set(vec);
  //   }

  //   const auto end = std::chrono::steady_clock::now();
  //   double dt_ms =
  //       std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(
  //           end - start)
  //           .count();
  //   dtPublisher.Set(dt_ms);

  //   static int i;
  //   i++;

  //   if (i % 10 == 3)
  //     trajectoryPub.Set(localizer.GetPoseHistory());

  //   inst.Flush();

  //   std::this_thread::sleep_for(30ms);
  // }

  return 0;
}
