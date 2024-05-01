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

#include "config.h"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <system_error>

#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>

void LocalizerConfig::print(std::string_view prefix) {
  fmt::println("{} root={}, rot={}, trans={}, cameras=[{}]", prefix,
               rootTableName, fmt::join(rotNoise, ", "),
               fmt::join(transNoise, ", "), fmt::join(cameras, ", "));
}

LocalizerConfig ParseConfig(std::string_view path) {
  std::error_code ec;
  std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
      wpi::MemoryBuffer::GetFile(path, ec);
  if (fileBuffer == nullptr || ec) {
    throw std::runtime_error(fmt::format("Cannot open file: {}", path));
  }

  wpi::json json = wpi::json::parse(fileBuffer->GetCharBuffer());

  return LocalizerConfig{
      .rootTableName = json.at("rootTableName").get<std::string>(),
      .rotNoise = json.at("rotNoise").get<std::vector<double>>(),
      .transNoise = json.at("transNoise").get<std::vector<double>>(),
      .cameras = json.at("cameras").get<std::vector<CameraConfig>>()};
}

void from_json(const wpi::json &json, CameraConfig &config) {
  config.m_subtableName = json.at("subtableName").get<std::string>();
  config.m_pixelNoise = json.at("pixelNoise").get<double>();
}
