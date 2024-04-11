#include "localizer.h"
#include "gtsam/nonlinear/Expression.h"
#include "gtsam/slam/expressions.h"
#include <algorithm>

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

    map<int, Pose3> worldTtags {
        {1, Pose3{ Rot3::RzRyRx(0, 0, 0), Point3{2, 0, 0} }}
        {2, Pose3{ Rot3::RzRyRx(0, 0, 0), Point3{2, 0, 0} }}
    };

    vector<Point3> WorldToCorners(int id) {
        auto maybePose = worldTtags.find(id);
        if (maybePose == worldTtags.end()) {
            // panic lol
            return {};
        }
        Pose3 worldTtag = maybePose->second;

        vector<Point3> out(4);
        std::transform(tagToCorners.begin(), tagToCorners.end(), out.begin(), &Pose3::transformFrom);

        return out;
    }
};

struct TagCorner
{
    Key landmarkID;
    Point3 worldToCorner;

    TagCorner(int tagID, int cornerID, Point3 p) : worldToCorner(p), landmarkID(TagIdToKey(tagID, cornerID)) {}
};

Localizer::Localizer(
    Cal3_S2 cameraCal, Pose3 bodyPcamera,
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
    auto worldPcorners = TagModel::WorldToCorners(tagID);

    for (size_t i = 0; i < NUM_CORNERS; i++)
    {
        auto key = TagIdToKey(tagID, i);

        // corner in image space
        Point2 measurement = corners[i];

        // current world->body pose
        const Pose3_  worldTbody_fac(state);
        // world->camera pose as a composition of world->body factory and body->camera factor
        const Pose3_ worldTcamera_fac = Pose3_(worldTbody_fac, &Pose3::transformPoseFrom, Pose3_(bodyPcamera));
        // Camera->tag corner vector
        const Point3_ camPcorner = transformTo(worldTcamera_fac, worldPcorners[i]);
        // project from vector down to pinhole location, then uncalibrate to pixel locations
        const Point2_ prediction = uncalibrate<Cal3_S2>(cameraCal, project(camPcorner));

        graph.addExpressionFactor(cameraNoise, measurement, prediction);
    }
}

Pose3 Localizer::Optimize()
{
    isam.update(graph, currentEstimate);

    // reset the graph; isam wants to be fed factors to be -added-
    graph.resize(0);
    currentEstimate.clear();

    return isam.calculateEstimate<Pose3>(currStateIdx);
}
