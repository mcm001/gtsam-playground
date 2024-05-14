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
#include <memory>
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
#include "camera_listener.h"
#include "config.h"
#include "config_listener.h"
#include "data_publisher.h"
#include "gtsam_utils.h"
#include "localizer.h"
#include "odom_listener.h"

using namespace gtsam;
using std::vector;
using namespace std::chrono_literals;

class LocalizerRunner {
private:
  std::shared_ptr<Localizer> localizer;
  OdomListener odomListener;
  DataPublisher dataPublisher;
  ConfigListener configListener;
  std::vector<CameraListener> cameraListeners;

  bool gotInitialGuess = false;

public:
  explicit LocalizerRunner(LocalizerConfig config)
      : localizer(std::make_shared<Localizer>()), odomListener{config},
        dataPublisher(config.rootTableName, localizer), configListener(config) {
    cameraListeners.reserve(config.cameras.size());
    for (const CameraConfig &camCfg : config.cameras) {
      cameraListeners.emplace_back(config.rootTableName, camCfg);
    }
  }

  void Update() {
    bool readyToOptimize = true;

    if (const auto prior = configListener.NewPosePrior()) {
      localizer->Reset(prior->value.pose, prior->value.noise, prior->time);
      gotInitialGuess = true;
    }

    if (const auto layout = configListener.NewTagLayout()) {
      TagModel::SetLayout(*layout);

      // Reset initial guess tracking since we got a new layout and our factors
      // are technically now wrong
      gotInitialGuess = false;
    }

    readyToOptimize &= gotInitialGuess;

    for (const auto &it : odomListener.Update()) {
      localizer->AddOdometry(it);
    }

    // localizer->Print("=========================\nAfter adding odometry
    // factors");

    for (auto &cam : cameraListeners) {
      bool ready = cam.ReadyToOptimize();
      readyToOptimize &= ready;

      if (ready) {
        for (const auto &it : cam.Update()) {
          localizer->AddTagObservation(it);
        }
      }
    }

    if (!readyToOptimize) {
      fmt::println("Not yet ready (see above) -- busywaiting");
      std::this_thread::sleep_for(1000ms);
      return;
    }

    // localizer->Print("=========================\nAfter adding vision
    // factors");

    try {
      localizer->Optimize();
      dataPublisher.Update();
      nt::NetworkTableInstance::GetDefault().Flush();
    } catch (const std::exception &e) {
      fmt::println("Exception optimizing: {}", e.what());
      localizer->Print();
      throw e;
    }
  }
};

int main(int argc, char **argv) {
  std::string configPath;
  if (argc == 1) {
    configPath = "test/resources/simulator.json";
  } else if (argc == 2) {
    configPath = argv[1];
  } else {
    fmt::println("Got an odd number of program arguments!");
    return -1;
  }

  fmt::println("Loading config from: {}", configPath);
  LocalizerConfig config = ParseConfig(configPath);
  config.print("Loaded config:");

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

  inst.StopServer();
  inst.SetServer(config.ntServerURI.c_str());
  inst.StartClient4("gtsam-meme");

  LocalizerRunner runner(config);

  while (true) {
    runner.Update();

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}
