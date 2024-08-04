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

#include <fmt/core.h>
#include <ntcore_cpp_types.h>

#include <variant>
#include <vector>

#include <wpi/DataLogReader.h>
#include <wpi/DenseMap.h>

#include "TagModel.h"

// The two new headers that allow using our Automatic Differentiation Expression
// framework
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>

// Header order is close to far
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

#include <frc/geometry/Twist3d.h>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

using ReplayInfoVariant = std::variant<TagDetection, frc::Twist3d>;

std::vector<nt::Timestamped<ReplayInfoVariant>>
GetTags(std::string_view path, std::string_view topicName) {
  using namespace wpi::log;

  std::error_code ec;
  auto buf = wpi::MemoryBuffer::GetFile(path, ec);
  if (ec) {
    throw std::runtime_error(fmt::format("Error: {}", ec.message()));
  }

  DataLogReader reader{std::move(buf)};

  wpi::DenseMap<int, StartRecordData *> entriesById;

  // from
  // https://github.com/wpilibsuite/allwpilib/blob/8c420fa4c1ddd88f8237c5464223a56685ae6dcf/glass/src/lib/native/cpp/support/DataLogReaderThread.cpp
  for (auto recordIt = reader.begin(), recordEnd = reader.end();
       recordIt != recordEnd; ++recordIt) {

    const DataLogRecord &record = *recordIt;
    if (record.IsStart()) {
      // Check if is start data
      StartRecordData data;
      if (record.GetStartData(&data)) {
        auto &entryPtr = entriesById[data.entry];
        if (entryPtr) {
          fmt::print("...DUPLICATE entry ID, overriding\n");
        }
        // auto [it, isNew] = entriesByName.emplace(data.name, data);
      } else {
        fmt::print("Start(INVALID)\n");
      }
    } else if (record.IsFinish()) {
      int entry;
      if (record.GetFinishEntry(&entry)) {
        auto it = entriesById.find(entry);
        if (it == entriesById.end()) {
          fmt::print("...ID not found\n");
        } else {
          entriesById.erase(it);
        }
      } else {
        fmt::print("Finish(INVALID)\n");
      }
    } else if (record.IsSetMetadata()) {
      MetadataRecordData data;
      if (record.GetSetMetadataData(&data)) {
        auto it = entriesById.find(data.entry);
        if (it == entriesById.end()) {
          fmt::print("...ID not found\n");
        } else {
          it->second->metadata = data.metadata;
        }
      } else {
        fmt::print("SetMetadata(INVALID)\n");
      }
    } else if (record.IsControl()) {
      fmt::print("Unrecognized control record\n");
    } else {
      // Check we know about this record

      int id = record.GetEntry();
      auto it = entriesById.find(id);
      if (it != entriesById.end()) {
        fmt::println("> {}", it->second->name);
        if (it->second->name == topicName) {
          fmt::println("hi");
        }
      } else {
        fmt::println("Unknown id {}", id);
      }
    }
  }

  return {};
}

int main() {
  Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
  Isotropic::shared_ptr measurementNoise =
      Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Create the set of ground-truth landmarks and poses
  vector<Point3> points = {}; // createPoints();
  vector<Pose3> poses = {};   // createPoses();

  // Create a factor graph
  ExpressionFactorGraph graph;

  // Specify uncertainty on first pose prior
  Vector6 sigmas;
  sigmas << Vector3(0.3, 0.3, 0.3), Vector3(0.1, 0.1, 0.1);
  Diagonal::shared_ptr poseNoise = Diagonal::Sigmas(sigmas);

  // x0 is an Expression, and we create a factor wanting it to be equal to
  // poses[0]
  // Specify that we measured x[0] at poses[0]
  Pose3_ x0('x', 0);
  graph.addExpressionFactor(x0, poses[0], poseNoise);

  // We create a constant Expression for the calibration here
  Cal3_S2_ cK(K);

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  for (size_t i = 0; i < poses.size(); ++i) {
    Pose3_ x('x', i);
    PinholeCamera<Cal3_S2> camera(poses[i], K);
    for (size_t j = 0; j < points.size(); ++j) {
      Point2 measurement = camera.project(points[j]);
      // Below an expression for the prediction of the measurement:
      Point3_ p('l', j);
      Point2_ prediction = uncalibrate(cK, project(transformTo(x, p)));
      // Again, here we use an ExpressionFactor
      graph.addExpressionFactor(prediction, measurement, measurementNoise);
    }
  }

  // Add prior on first point to constrain scale, again with ExpressionFactor
  Isotropic::shared_ptr pointNoise = Isotropic::Sigma(3, 0.1);
  graph.addExpressionFactor(Point3_('l', 0), points[0], pointNoise);

  // Create perturbed initial
  Values initial;
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
  for (size_t i = 0; i < poses.size(); ++i)
    initial.insert(Symbol('x', i), poses[i].compose(delta));
  for (size_t j = 0; j < points.size(); ++j)
    initial.insert<Point3>(Symbol('l', j),
                           points[j] + Point3(-0.25, 0.20, 0.15));
  cout << "initial error = " << graph.error(initial) << endl;

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initial).optimize();
  cout << "final error = " << graph.error(result) << endl;

  return 0;
}
