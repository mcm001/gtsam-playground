# Setup 

One-line cmake incantation

```
cmake -B build -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

# Running a photon sim example

[This sim example](https://github.com/PhotonVision/champs_2024/tree/gtsam-testing/sim_projects/apriltag_yaw_only) in the gtsam-testing branch is set up to provide simulated data to the current build of gtsam-playground. Just run it as a robot simulation project! Until I get CLI working, the gtsam node NT server IP address will need to be manually changed per-machine this is tested on.

I visualize output data usually using advantagescope. The 3d visualizer is great.

# NT API

As of right now, these are our publishers/subscribers. The NT4-provided set timestamp is used for latency compensation. The list of camera names is configured by changing the config JSON, located at a hard-coded path relative to the current working directory: currently, `test/resources/simulator.json`. The JSON example below shows two cameras with given tag corner pixel standard deviations, plus global odometry standard deviations on rotation and translation.

```json
{
    "rootTableName": "gtsam_meme_localizer1",
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

# Packages

You'll probably need to additionally install these packages:

```
libboost-all-dev protobuf-compiler ninja-build
```

# Notes

WPILib uses a version of Eigen from https://github.com/wpilibsuite/allwpilib/blob/main/upstream_utils/update_eigen.py#L100 SHA is 96880810295b65d77057f4a7fb83a99a590122ad
