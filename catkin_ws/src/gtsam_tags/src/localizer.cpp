#include "localizer.h"

using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::P;
using symbol_shorthand::X;

constexpr int NUM_CORNERS = 4;

static gtsam::Key TagIdToKey(int tagID, int corner)
{
    // L-shorthand for landmark
    return L(NUM_CORNERS * tagID + corner);
}

namespace TagModel
{
    float width = 0.15; // TODO random guess
    vector<Point3> tagToCorners{
        {-width / 2.0, width / 2.0, 0},
        {width / 2.0, width / 2.0, 0},
        {width / 2.0, -width / 2.0, 0},
        {-width / 2.0, -width / 2.0, 0},
    };
};

struct TagCorner
{
    Key landmarkID;
    Point3 worldToCorner;

    TagCorner(int tagID, int cornerID, Point3 p) : worldToCorner(p), landmarkID(TagIdToKey(tagID, cornerID)) {}
};

Localizer::Localizer(
    Cal3_S2::shared_ptr cameraCal, Pose3 bodyPcamera,
    SharedNoiseModel cameraNoise,
    SharedNoiseModel odometryNoise,
    SharedNoiseModel posePriorNoise,
    Pose3 initialPose)
    : cameraCal(cameraCal), bodyPcamera(bodyPcamera), cameraNoise(cameraNoise), odometryNoise(odometryNoise), posePriorNoise(posePriorNoise)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.cacheLinearizedFactors = false;
    parameters.enableDetailedResults = true;
    parameters.print();

    isam = ISAM2{parameters};

    // Anchor graph using initial pose
    graph.addPrior(X(0), initialPose, posePriorNoise);
    currentEstimate.insert(X(0), initialPose);
}

void Localizer::AddOdometry(Pose3 twist)
{
    currStateIdx++;

    // Add an odometry twist from our last state to our new one
    graph.emplace_shared<BetweenFactor<Pose3>>(currStateIdx - 1, currStateIdx, twist, odometryNoise);
    // And get initial guess just by composing previous pose
    currentEstimate.insert(currStateIdx, isam.calculateEstimate<Pose3>(currStateIdx).transformPoseFrom(twist));
}

void Localizer::AddTagObservation(Key state, int tagID, vector<Point2> corners)
{
    for (size_t i = 0; i < NUM_CORNERS; i++) {
        auto key = TagIdToKey(tagID, i);
        auto landmarkEntry = landmarks.find(key);
        SmartFactor::shared_ptr l;

        if (landmarkEntry == landmarks.end()) {
            // Create a new factor for this corner
            l = SmartFactor::shared_ptr(new SmartFactor(cameraNoise, cameraCal, bodyPcamera));
            landmarks[key] = l;

            graph.push_back(l);
        } else {
            l = landmarks[key];
        }

        // we now have a valid landmark identified
        auto measurement = corners[i];
        l->add(measurement, state);
    }
}

Pose3 Localizer::Optimize()
{
    return isam.calculateEstimate<Pose3>(currStateIdx);
}
