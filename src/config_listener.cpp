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

#include "config_listener.h"

#include <networktables/NetworkTableInstance.h>

ConfigListener::ConfigListener(LocalizerConfig config)
    : layoutSub(nt::NetworkTableInstance::GetDefault()
                    .GetStringTopic(config.rootTableName + "/input/tag_layout")
                    .Subscribe({}, {.pollStorage = 1})),
      initialGuessSub(
          nt::NetworkTableInstance::GetDefault()
              .GetStructTopic<frc::Pose3d>(config.rootTableName +
                                           "/input/pose_initial_guess")
              .Subscribe({}, {
                                 .pollStorage = 1,
                                 .sendAll = true,
                                 .keepDuplicates = true,
                             })) {}

std::optional<frc::AprilTagFieldLayout> ConfigListener::NewTagLayout() {
  const auto newLayouts = layoutSub.ReadQueue();
  if (newLayouts.size()) {
    wpi::json json = wpi::json::parse(newLayouts.back().value);
    return json.get<frc::AprilTagFieldLayout>();
  }

  return std::nullopt;
}

std::optional<Timestamped<Pose3WithNoise>> ConfigListener::NewPosePrior() {
  const auto newGuesses = initialGuessSub.ReadQueue();
  if (newGuesses.size()) {
    return Timestamped<Pose3WithNoise>{
        static_cast<uint64_t>(newGuesses.back().time),
        Pose3dToGtsamPose3(newGuesses.back().value)};
  }

  return std::nullopt;
}
