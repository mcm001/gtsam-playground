/**
 * @file ISAM2Example_SmartFactor.cpp
 * @brief test of iSAM with smart factors, led to bitbucket issue #367
 * @author Alexander (pumaking on BitBucket)
 */

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::P;
using symbol_shorthand::X;
using symbol_shorthand::L;

// Make the typename short so it looks much cleaner
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
using LandmarkMap = map<Key, SmartFactor::shared_ptr>;

constexpr int NUM_CORNERS = 4;

gtsam::Key TagIdToKey(int tagID, int corner) {
  return L(NUM_CORNERS * tagID + corner);
}

struct TagCorner {
  Key landmarkID;
  Point3 worldToCorner;

  TagCorner(int tagID, int cornerID, Point3 p) : worldToCorner(p), landmarkID(TagIdToKey(tagID, cornerID)) {}
};

void addFakeObservations(LandmarkMap &landmarks, Key state, vector<TagCorner> &tagCorners, Pose3 &pose, Cal3_S2::shared_ptr K, Pose3 bodyToCamera) {
  for (const auto& p : tagCorners) {
    auto landmarkPair = landmarks.find(p.landmarkID);
    if (landmarkPair != landmarks.end()) {
      auto worldToCam = pose.compose(bodyToCamera);
      auto relative = worldToCam.transformTo(p.worldToCorner);
      cout 
        << "observation " 
        << worldToCam.x() 
        << "," << worldToCam.y()
        << "," << worldToCam.z()
        << " landmark " 
        << p.worldToCorner.x() 
        << "," << p.worldToCorner.y()
        << "," << p.worldToCorner.z()
        << " relative " 
        << relative.x() 
        << "," << relative.y()
        << "," << relative.z() << endl;
      

      auto measurement = PinholePose<Cal3_S2>(worldToCam, K).project(p.worldToCorner);
      cout << "State " << state << ": landmark " << p.landmarkID << " fake-observed at " << measurement.x() << "," << measurement.y() << endl;
      landmarkPair->second->add(measurement, state);
    } else {
      cerr << "Factor " << p.landmarkID << " not found in landmark map?" << endl;
    }
  }
}

/**
 * This test does a couple things:
 * - has a camera located at a known pose relative to the body
 * - camera has known calibration and pixel noise
 * - we start at a known pose (if i don't i get an indeterminant system; 
 *   need more than one pixel observation on first loop to fully determine our pose3)
 * - between states we use odometry BetweenFactors (which I -think- act similar to twists)
 * - at each timestamp we observe a fixed point in space through our camera
*/
int main(int argc, char* argv[]) {
  // approx a 1080p c920
  Cal3_S2::shared_ptr K(new Cal3_S2(1300, 1300, 0, 1000, 500));

  auto measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  Vector6 sigmas;
  sigmas << Vector3::Constant(0.1), Vector3::Constant(0.3);
  auto posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

  // odometry noise 
  Vector6 odomSigma;
  odomSigma << Vector3::Constant(0.001), Vector3::Constant(0.05);
  auto odometryNoise = noiseModel::Diagonal::Sigmas(odomSigma);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  parameters.print();
  ISAM2 isam(parameters);

  // Create a factor graph
  NonlinearFactorGraph graph;
  Values initialEstimate;

  vector<TagCorner> tagCorners {
    {1, 0, {1, 2, 0}},
    // {1, 0, {1, 1, 1}},
    // {1, 1, {0, 0, 1}},
    // {1, 2, {0.1, 0, 1}},
    // {1, 3, {0, 0.1, 1}},
    // {2, 0, {0, 2, 0}},
  };

  // Intentionally initialize the variables off from the ground truth
  Pose3 delta(Rot3::Rodrigues(0.0, 0.0, 0.0), Point3(0.05, -0.10, 0.20));

  // Ground truth poses for our robot's trajectory
  // we strafe in +y (so left in the robot frame)
  Pose3 pose1(Rot3(), Point3(0.0, 0.0, 0.0));
  Pose3 pose2(Rot3(), Point3(0, .2, 0.0));
  Pose3 pose3(Rot3(), Point3(0, .4, 0.0));
  Pose3 pose4(Rot3(), Point3(0, .5, 0.0));
  Pose3 pose5(Rot3(), Point3(0, .6, 0.0));

  vector<Pose3> poses = {pose1, pose2, pose3, pose4, pose5};

  // Add first pose
  graph.addPrior(X(0), poses[0], posePriorNoise);
  initialEstimate.insert(X(0), poses[0].compose(delta));

  // body-camera transform
  // Camera +Z points out of the camera, so rotate camera 90 about +x so we have a camera pointing out the left of the robot
  Pose3 body_P_sensor {Rot3::Rx(-90 * 3.141592 / 180.0), Point3{}};

  // Each landmark we can observe multiple times needs its own smartfactor
  LandmarkMap landmarks;
  for (const auto& p : tagCorners) {
    landmarks[p.landmarkID] = SmartFactor::shared_ptr(new SmartFactor(measurementNoise, K, body_P_sensor));

    // Not sure if this is legal to go before adding the first factor?
    graph.push_back(landmarks[p.landmarkID]);
  }

  // Create smart factor with measurement from first pose only
  addFakeObservations(landmarks, X(0), tagCorners, poses[0], K, body_P_sensor);

  // loop over remaining poses
  for (size_t i = 1; i < 5; i++) {
    cout << "****************************************************" << endl;
    cout << "i = " << i << endl;

    // Add odometry factor connecting us to the previous node

    Pose3 twist = poses[i-1].transformPoseTo(poses[i]);
    cout << "Twist= " << twist << endl;
    graph.emplace_shared<BetweenFactor<Pose3>>(X(i-1), X(i), twist, odometryNoise);
    initialEstimate.insert(X(i), poses[i].compose(delta));


    // "Simulate" measurements from this pose
    addFakeObservations(landmarks, X(i), tagCorners, poses[i], K, body_P_sensor);

    // Update iSAM2
    ISAM2Result result = isam.update(graph, initialEstimate);
    cout << "Isam2 results: ";
    result.print();

    // cout << "> Detailed results:" << endl;
    // for (auto keyedStatus : result.detail->variableStatus) {
    //   const auto& status = keyedStatus.second;
    //   PrintKey(keyedStatus.first);
    //   cout << " {" << endl;
    //   cout << "reeliminated: " << status.isReeliminated << endl;
    //   cout << "relinearized above thresh: " << status.isAboveRelinThreshold
    //        << endl;
    //   cout << "relinearized involved: " << status.isRelinearizeInvolved << endl;
    //   cout << "relinearized: " << status.isRelinearized << endl;
    //   cout << "observed: " << status.isObserved << endl;
    //   cout << "new: " << status.isNew << endl;
    //   cout << "in the root clique: " << status.inRootClique << endl;
    //   cout << "}" << endl;
    // }

    Values currentEstimate = isam.calculateEstimate();
    currentEstimate.print("Current estimate: ");

    for (auto [key, smartFactor] : landmarks) {
      cout << "Landmark " << key << " point estimate" << endl;
      boost::optional<Point3> pointEstimate = smartFactor->point(currentEstimate);
      if (pointEstimate) {
        cout << *pointEstimate << endl;
      } else {
        cout << "Point degenerate." << endl;
      }
    }

    // Reset graph and initial estimate for next iteration
    graph.resize(0);
    initialEstimate.clear();
  }

  return 0;
}
