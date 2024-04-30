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

#include "camera_listener.h"

#include <networktables/NetworkTableInstance.h>

#include "localizer.h"

using std::vector;
using namespace gtsam;

CameraListener::CameraListener(CameraConfig config,
                               std::shared_ptr<Localizer> localizer_)
    : localizer(localizer_),
      tagSub(nt::NetworkTableInstance::GetDefault()
                 .GetStructArrayTopic<TagDetection>("/cam/tags")
                 .Subscribe({},
                            {
                                .pollStorage = 100,
                                .sendAll = true,
                                .keepDuplicates = true,
                            })),
      measurementNoise(noiseModel::Isotropic::Sigma(2, config.m_pixelNoise)) {}

void CameraListener::Update() {
  if (!localizer) {
    throw std::runtime_error("Localizer was null");
  }

  const auto tags = tagSub.ReadQueue();

  // For each tag-array in the queue
  for (const auto &tarr : tags) {
    // For each tag in this tag array
    for (const auto &t : tarr.value) {
      vector<Point2> cornersForGtsam(4);
      for (const auto &c : t.corners) {
        cornersForGtsam.emplace_back(c.first, c.second);
      }

      try {
        localizer->AddTagObservation(t.id, cornersForGtsam,
                                     tarr.time);
      } catch (std::exception e) {
        fmt::println("exception adding tag, {}", e.what());
      }
    }
  }
}
