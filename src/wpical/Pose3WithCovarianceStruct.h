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

#include <iostream>

#include "Pose3WithCovariance.h"

template <> struct WPILIB_DLLEXPORT wpi::Struct<Pose3WithCovariance> {
  static constexpr size_t poseSize = wpi::Struct<frc::Pose3d>::GetSize();

  static constexpr std::string_view GetTypeName() {
    return "Pose3WithCovariance";
  }

  static constexpr size_t GetSize() { return poseSize + (36 * sizeof(double)); }

  static constexpr std::string_view GetSchema() {
    return "Pose3d pose; double covariance[36]";
  }

  static Pose3WithCovariance Unpack(std::span<const uint8_t> data) {
    // row-major mat
    auto cov_ = wpi::UnpackStruct<std::array<double, 36>, poseSize>(data);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(cov_.data());

    return Pose3WithCovariance{wpi::UnpackStruct<frc::Pose3d, 0>(data), cov};
  }

  static void Pack(std::span<uint8_t> data, const Pose3WithCovariance &value) {
    wpi::PackStruct<0>(data, value.pose);

    std::cout << "Cov in struct\n" << value.covariance << std::endl;

    // Make sure our matrix is row-major. Not that it matters - this matrix is
    // symetric
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covMap{value.covariance};

    std::array<double, 36> covArr;
    auto covSpan = std::span<double, 36>{covMap.data(), covMap.data() + 36};
    std::copy(covSpan.begin(), covSpan.end(), covArr.begin());

    wpi::PackStruct<poseSize>(data, covArr);
  }

  static void
  ForEachNested(std::invocable<std::string_view, std::string_view> auto fn) {
    wpi::ForEachStructSchema<frc::Pose3d>(fn);
  }
};

static_assert(wpi::StructSerializable<Pose3WithCovariance>);
