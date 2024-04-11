#pragma once

#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include "gtsam/slam/expressions.h"

using namespace gtsam;
using std::map;
using std::vector;
using symbol_shorthand::X;

class Localizer
{
    using SmartFactor = SmartProjectionPoseFactor<Cal3_S2>;
    using LandmarkMap = map<Key, SmartFactor::shared_ptr>;

public:
    Localizer(
        Cal3_S2_ cameraCal,
        Pose3 bodyTcamera,
        SharedNoiseModel cameraNoise,
        SharedNoiseModel odometryNoise,
        SharedNoiseModel posePriorNoise,
        Pose3 initialPose);

    void AddOdometry(Pose3 twist);
    void AddTagObservation(Key state, int tagID, vector<Point2> corners);

    Pose3 Optimize();

private:
    Cal3_S2_ cameraCal;
    Pose3 bodyPcamera;
    SharedNoiseModel cameraNoise;
    SharedNoiseModel odometryNoise;
    SharedNoiseModel posePriorNoise;

    ExpressionFactorGraph graph;
    Values currentEstimate;

    ISAM2 isam;

public:
    // keep track of our current state 
    Key currStateIdx = X(0);

    // keep track of smart factors for each landmark (one per tag corner)
    LandmarkMap landmarks;
};
