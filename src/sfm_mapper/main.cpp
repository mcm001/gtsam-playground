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

#include <variant>
#include <vector>

#include <wpi/DataLogReader.h>
#include <wpi/DenseMap.h>

#include "TagModel.h"

#include <ntcore_cpp_types.h>

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
  for (auto recordIt = m_reader.begin(), recordEnd = m_reader.end();
       recordIt != recordEnd; ++recordIt) {

    if (record.IsStart()) {
      // Check if is start data
      StartRecordData data;
      if (record.GetStartData(&data)) {
        auto &entryPtr = entriesById[data.entry];
        if (entryPtr) {
          wpi::print("...DUPLICATE entry ID, overriding\n");
        }
        auto [it, isNew] = m_entriesByName.emplace(data.name, data);
      } else {
        wpi::print("Start(INVALID)\n");
      }
    } else if (record.IsFinish()) {
      int entry;
      if (record.GetFinishEntry(&entry)) {
        auto it = entriesById.find(entry);
        if (it == entriesById.end()) {
          wpi::print("...ID not found\n");
        } else {
          entriesById.erase(it);
        }
      } else {
        wpi::print("Finish(INVALID)\n");
      }
    } else if (record.IsSetMetadata()) {
      MetadataRecordData data;
      if (record.GetSetMetadataData(&data)) {
        auto it = entriesById.find(data.entry);
        if (it == entriesById.end()) {
          wpi::print("...ID not found\n");
        } else {
          it->second->metadata = data.metadata;
        }
      } else {
        wpi::print("SetMetadata(INVALID)\n");
      }
    } else if (record.IsControl()) {
      wpi::print("Unrecognized control record\n");
    } else {
      // Check we know about this record

      int id = record.GetEntry();
      auto it = entriesById.find(id);
      if (it != entriesById.end()) {
        fmt::println("> {}", it->name);
        if (it->name == topicName) {
          fmt::println("hi");
        }
      } else {
        fmt::println("Unknown id {}", id);
      }
    }
  }

  return {};
}

int main() { return 0; }
