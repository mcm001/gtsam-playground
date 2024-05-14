# Setup

These instructions are intended for use on Linux (including WSL) only.

You'll probably need to install these packages:

```
build-essential cmake libboost-all-dev protobuf-compiler ninja-build
```

With these packages installed, configure using CMake. I'm using Ninja, but any generator will work.

```
cmake -B build -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

For Windows, use this instead: `cmake -B build/ -S . -DCMAKE_TOOLCHAIN_FILE="vcpkg/scripts/buildsystems/vcpkg.cmake" -DCMAKE_BUILD_TYPE=RelWithDebInfo`

Depending on your RAM, you may need to use `-j <number>` to control the number of jobs when building.

And build and run the actual executable! This will take a while

```
clear && cmake --build build --target gtsam-node && ./build/bin/gtsam-node
```

# Running a photon sim example

[This sim example](https://github.com/PhotonVision/champs_2024/tree/gtsam-testing/sim_projects/apriltag_yaw_only) in the gtsam-testing branch is set up to provide simulated data to the current build of gtsam-playground. Just run it as a robot simulation project! The NT server URI is set by changing the JSON below. If you're running this in WSL2, the URI must be set to the IP address of your computer in **Windows**.

I visualize output data usually using advantagescope. The 3d visualizer is great.

In order for the data transfer to work properly, start advantagescope, then sim, then run gtsam-node. The /SmartDashboard/VisionSystemSim-main/Sim Field/Gtsam Robot topic will show the fused pose of the robot.

# NT API

As of right now, these are our publishers/subscribers. The NT4-provided set timestamp is used for latency compensation. The list of camera names is configured by changing the config JSON. If no arguemnts are provided, the config is pulled from the file `test/resources/simulator.json`. If one argument is provided to gtsam-node, then that arguemnt is used as the path instead. The JSON example below shows two cameras with given tag corner pixel standard deviations, plus global odometry standard deviations on rotation and translation.

A [reference implementation from robot code](https://github.com/PhotonVision/champs_2024/blob/gtsam-testing/sim_projects/apriltag_yaw_only/src/main/java/frc/robot/GtsamInterface.java) is also available.

```json
{
    "rootTableName": "gtsam_meme_localizer1",
    "ntServerURI": "10.TE.AM.2",
    "cameras": [
        {
            "subtableName": "sim_camera1",
            "pixelNoise": 10
        },
        {
            "subtableName": "sim_camera2",
            "pixelNoise": 12
        }
    ],
    "rotNoise": [ 0.087263889, 0.087263889, 0.087263889 ],
    "transNoise": [ 0.001, 0.001, 0.001 ]
}

```

Subscribers

| Topic                                     | Type                  | Remark                                                                            |
|-------------------------------------------|-----------------------|-----------------------------------------------------------------------------------|
| {root}/{camera name}/input/tags           | [struct:TagDetection[]](https://github.com/PhotonVision/champs_2024/blob/gtsam-testing/sim_projects/apriltag_yaw_only/src/main/java/frc/robot/TagDetectionStruct.java) | List of currently observed tags. The image coordinates must be un-distorted first |
| {root}/{camera name}/input/robotTcam      | struct:Transform3d    | Current robot->camera transform                                                   |
| {root}/{camera name}/input/cam_intrinsics | double[]              | Camera pinhole-only intrinsics, order must be [fx fy cx cy]                       |
| {root}/input/odom_twist                   | struct:Twist3d        | Twist from the last timestamp to now                                              |

Publishers

| Topic                        | Type            | Remark                                                                         |
|------------------------------|-----------------|--------------------------------------------------------------------------------|
| {root}/output/optimized_pose | struct:Pose3d   | The optimized pose/its timestamp                                               |
| {root}/output/optimized_traj | struct:Pose3d[] | List of (some subset of) optimized past poses over time                        |
| {root}/output/pose_stddev    | double[]        | Standard deviation of most recent optimized pose. Order is [rx ry rz tx ty tz] |

# Notes

WPILib uses a version of Eigen from https://github.com/wpilibsuite/allwpilib/blob/main/upstream_utils/update_eigen.py#L100 SHA is 96880810295b65d77057f4a7fb83a99a590122ad
