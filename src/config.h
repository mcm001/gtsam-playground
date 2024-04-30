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

#pragma once

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <string>
#include <vector>

#include <wpi/json.h>

struct CameraConfig {
  std::string m_subtableName;

  double m_pixelNoise;
};

// Print CameraConfigs using fmtlib
template <> struct fmt::formatter<CameraConfig> : formatter<string_view> {
  auto format(CameraConfig const &c, format_context &ctx) const {
    // return formatter<string_view>::format(c.m_subtableName, ctx);
    return fmt::format_to(
        ctx.out(), "(name={}, pixel={}, rot=[{}], trans=[{}])",
        c.m_subtableName, c.m_pixelNoise, fmt::join(c.m_rotNoise, ", "),
        fmt::join(c.m_transNoise, ", "));
  }
};

struct LocalizerConfig {
  std::string rootTableName = "/gtsam_meme/"
  std::vector<CameraConfig> cameras;

  void print(std::string_view prefix = "");

  // odometry noise factors
  std::vector<double> rotNoise;
  std::vector<double> transNoise;
};

LocalizerConfig ParseConfig(std::string_view path);

void from_json(const wpi::json &json, CameraConfig &config);
