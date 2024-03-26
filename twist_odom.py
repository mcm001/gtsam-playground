"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robot motion example, with prior and two odometry measurements
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function
import random

import gtsam
from gtsam.symbol_shorthand import X, L

import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np

from wpimath.geometry import Pose2d, Twist2d, Rotation2d

TAG_POSE = Pose2d(2, 0, 0)

def generate_robot_one_tag():

    # initial state
    poses = [Pose2d(0, 0, 0)]
    twists = [Twist2d(0, 0, 0)]
    groundTruthObservations = [TAG_POSE.relativeTo(poses[-1])]

    dt = 0.02

    for i in range(200):
        pose: Pose2d = poses[-1]

        dx = 1 * dt
        dy = 0 * dt
        dtheta = 0 * dt

        twist = Twist2d(dx, dy, dtheta)
        newpose = pose.exp(twist)

        poses.append(newpose)
        twists.append(twist)
        groundTruthObservations.append(TAG_POSE.relativeTo(poses[-1]))

    return (poses, twists, groundTruthObservations)
    
def wpi_to_gtsam_pose2(pose: Pose2d):
    return gtsam.Pose2(pose.x, pose.y, pose.rotation().radians())

# Create noise models

# Noise in our initial pose estimate
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
# Noise added per predict step
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.01]))
# noise in our vision measurement
VISION_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([.001, .001, 0.04]))


def main():
    
    (poses, twists, solvepnpPoses) = generate_robot_one_tag()

    # Create a Factor Graph and Values to hold the new data
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # vision pose tracking
    tag_idx = 0
    tag_pose_gtsam = wpi_to_gtsam_pose2(TAG_POSE)
    graph.add(gtsam.PriorFactorPose2(L(0), tag_pose_gtsam, None))    
    initial.insert(L(tag_idx), tag_pose_gtsam)

    # Add a prior on the first pose, setting it to the origin
    # A prior factor consists of a mean and a noise model (covariance matrix)
    priorMean = wpi_to_gtsam_pose2(poses[0])  # prior at origin
    graph.add(gtsam.PriorFactorPose2(X(0), priorMean, PRIOR_NOISE))    

    # Add odometry factors
    for (i, (pose, twist, robotToTag)) in enumerate(zip(poses, twists, solvepnpPoses)):
        
        # initial guess but corrupted
        noiseyPose = wpi_to_gtsam_pose2(pose)
        noiseyPose = gtsam.Pose2(
            noiseyPose.x() + (random.random() - 0.5) * 0.1,
            noiseyPose.y() + (random.random() - 0.5) * 0.1,
            noiseyPose.theta() + (random.random() - 0.5) * 0.05,
        )
        initial.insert(X(i), noiseyPose)
        

        # Add twists between pose i and i+1
        if i > 0:
                odom_twist = gtsam.Pose2(twist.dx, twist.dy, twist.dtheta)
                graph.add(gtsam.BetweenFactorPose2(X(i-1), X(i), odom_twist, ODOMETRY_NOISE))

        if (i % 4) == 0:
            # And add vision measurement
            vision_dpose = wpi_to_gtsam_pose2(robotToTag)
            graph.add(gtsam.BetweenFactorPose2(X(i), L(tag_idx), vision_dpose, VISION_NOISE))


    # optimize using Levenberg-Marquardt optimization
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

    # 5. Calculate and print marginal covariances for all variables
    marginals = gtsam.Marginals(graph, result)
    for i in range(0, len(poses)):
        print("X{} covariance:\n{}\n".format(i,
                                             marginals.marginalCovariance(X(i))))

    print(graph)

    for i in range(0, len(poses)):
        gtsam_plot.plot_pose2(0, result.atPose2(X(i)), 0.1,
                            None)
                            #   marginals.marginalCovariance(X(i)))
    
    for pose in poses:
        plt.scatter(pose.x, pose.y)

    plt.axis('equal')
    plt.xlabel("X, meters")
    plt.ylabel("Y, meters")
    plt.title("Robot pose and marginal covariance\nTag at: (2,0)")
    
    plt.figure(1)
    covs = []
    for i in range(0, len(poses)):
        cov = marginals.marginalCovariance(X(i))
        w, v = np.linalg.eigh(cov)
        norm = np.linalg.norm(w)
        covs.append(norm)
    plt.plot(covs)
    plt.title("Total covariance vs pose idx")


    plt.show()


if __name__ == "__main__":
    main()
