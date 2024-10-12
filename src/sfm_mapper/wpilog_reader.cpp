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

#include "wpilog_reader.h"

#include <fmt/core.h>

#include <optional>
#include <variant>
#include <vector>

#include <frc/geometry/struct/Twist3dStruct.h>
#include <wpi/DataLogReader.h>
#include <wpi/DenseMap.h>

#include "TagDetectionStruct.h"
#include "TagModel.h"

using namespace frc;
using namespace wpi::log;

using ReplayInfoVariant = std::variant<TagDetection, Twist3d>;

wpilog_reader::LogData
wpilog_reader::LoadDataFile(std::string_view path, std::string_view odomTopic,
                            std::string_view cam1_topic) {

  std::error_code ec;
  auto buf = wpi::MemoryBuffer::GetFile(path, ec);
  if (ec) {
    throw std::runtime_error(fmt::format("Error: {}", ec.message()));
  }

  DataLogReader reader{std::move(buf)};

  std::optional<StartRecordData> odomStartData;
  std::optional<StartRecordData> cam1StartData;
  LogData ret;

  // from
  // https://github.com/wpilibsuite/allwpilib/blob/8c420fa4c1ddd88f8237c5464223a56685ae6dcf/glass/src/lib/native/cpp/support/DataLogReaderThread.cpp
  for (auto recordIt = reader.begin(), recordEnd = reader.end();
       recordIt != recordEnd; ++recordIt) {

    auto &record = *recordIt;
    if (record.IsStart()) {
      // Check if is start data
      StartRecordData data;
      if (record.GetStartData(&data)) {
        // fmt::println("Start({})", data.name);

        // Check if this matches either of our topics
        if (data.name == odomTopic) {
          fmt::println("Start(odom)");
          odomStartData = data;
        } else if (data.name == cam1_topic) {
          fmt::println("Start(cam1)");
          cam1StartData = data;
        } else {
          // fmt::print("Start(UNKNOWN)\n");
        }

      } else {
        fmt::print("Start(INVALID)\n");
      }
    } else if (record.IsFinish()) {
      int entry;
      if (record.GetFinishEntry(&entry)) {
        fmt::print("Finish(TODO)\n");
        // auto it = entriesById.find(entry);
        // if (it == entriesById.end()) {
        //   fmt::print("...ID not found\n");
        // } else {
        //   entriesById.erase(it);
        // }
      } else {
        fmt::print("Finish(INVALID)\n");
      }
    } else if (record.IsSetMetadata()) {
      MetadataRecordData data;
      if (record.GetSetMetadataData(&data)) {
        fmt::print("SetMetadata(TODO)\n");
      } else {
        fmt::print("SetMetadata(INVALID)\n");
      }
    } else if (record.IsControl()) {
      fmt::print("Unrecognized control record\n");
    } else {

      // Check we know about this record
      int id = record.GetEntry();
      if (odomStartData && odomStartData->entry == id) {
        fmt::print("Data(ODOM)\n");
        wpi::UnpackStruct<Twist3d>(record.GetRaw());
      }
      if (cam1StartData && cam1StartData->entry == id) {
        fmt::print("Data(CAM1)\n");
        wpi::UnpackStruct<TagDetection>(record.GetRaw());
      }
    }
  }

  return ret;
}

int main() { return 0; }
