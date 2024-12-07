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

#include <gtest/gtest.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "sfm_mapper/helpers.h"

static frc::AprilTagFieldLayout tagLayoutGuess =
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2024Crescendo);

TEST(HelpersTest, PoseEst) {
  // [{"id":8,"corners":[{"x":436.5589785234008,"y":251.15456242690655},{"x":457.47607000155523,"y":251.51183531387946},{"x":457.8874053801833,"y":230.79771635636783},{"x":437.05512166830766,"y":230.2915431029345}]},{"id":7,"corners":[{"x":507.2061782424747,"y":250.85231240802392},{"x":527.7365607155515,"y":251.18118248213685},{"x":527.9621429707333,"y":229.87290714048356},{"x":507.37257606632056,"y":230.0130995737003}]}]

  std::vector<TagDetection> tags{
      {.id = 8,
       .corners = {{
                       436.5589785234008,
                       251.15456242690655,
                   },
                   {
                       457.47607000155523,
                       251.51183531387946,
                   },
                   {
                       457.8874053801833,
                       230.79771635636783,
                   },
                   {437.05512166830766, 230.2915431029345}}},
      {

          .id = 7,
          .corners = {{
                          507.2061782424747,
                          250.85231240802392,
                      },
                      {
                          527.7365607155515,
                          251.18118248213685,
                      },
                      {
                          527.9621429707333,
                          229.87290714048356,
                      },
                      {507.37257606632056, 230.0130995737003}}

      }};

  estimateWorldTcam(tags, tagLayoutGuess);
}
