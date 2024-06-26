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

#include "data_publisher.h"

#include <networktables/NetworkTableInstance.h>

#include "gtsam_utils.h"
#include "localizer.h"

using std::vector;
using namespace gtsam;

DataPublisher::DataPublisher(std::string rootTable,
                             std::shared_ptr<Localizer> localizer_)
    : localizer(localizer_),
      optimizedPosePub(
          nt::NetworkTableInstance::GetDefault()
              .GetStructTopic<frc::Pose3d>(rootTable + "/output/optimized_pose")
              .Publish({
                  .sendAll = true,
                  .keepDuplicates = true,
              })),
      trajectoryHistoryPub(nt::NetworkTableInstance::GetDefault()
                               .GetStructArrayTopic<frc::Pose3d>(
                                   rootTable + "/output/optimized_traj")
                               .Publish({
                                   .sendAll = true,
                                   .keepDuplicates = true,
                               })),
      stdDevPub(nt::NetworkTableInstance::GetDefault()
                    .GetDoubleArrayTopic(rootTable + "/output/pose_stddev")
                    .Publish({
                        .sendAll = true,
                        .keepDuplicates = true,
                    })) {}

void DataPublisher::Update() {
  if (!localizer) {
    throw std::runtime_error("Localizer was null");
  }

  auto time = localizer->GetLastOdomTime();

  {
    auto est = localizer->GetLatestWorldToBody();
    optimizedPosePub.Set(GtsamToFrcPose3d(est), time);
  }
  {
    auto mat = localizer->GetPoseComponentStdDevs();
    std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    stdDevPub.Set(vec, time);
  }
  {
    static int i;
    i++;

    if (i % 3 == 2)
      trajectoryHistoryPub.Set(localizer->GetPoseHistory());
  }
}
